#ifndef COMMANDPROCESSOR_H_
#define COMMANDPROCESSOR_H_

/**
 * @file commandProcessor.h
 * @brief A lightweight serial command dispatcher for Arduino-based systems.
 *
 * Design overview
 * ---------------
 * Commands are ASCII strings sent over one or more Stream objects (hardware
 * serial ports, USB CDC, software serial, etc.).  Each command follows a
 * simple get/set convention:
 *
 *   GCMD          — "get" the value associated with CMD
 *   SCMD,<value>  — "set" the value associated with CMD to <value>
 *
 * A Command table entry whose `cmd` field starts with '?' accepts either
 * form (G or S prefix).  Entries without '?' are matched literally.
 *
 * Multiple Command tables can be registered as a singly-linked CommandList.
 * Commands are dispatched to strongly-typed handler paths (int, float,
 * bool, string, function …) controlled by the CmdTypes enum.
 *
 * Known limitations / to-dos
 * --------------------------
 *  - Stream array is fixed at MAX_STREAMS (5) entries.
 *  - Only one commandProcessor instance is supported at a time because the
 *    built-in `listCommands` / `listCommand` helpers reference a module-level
 *    pointer (CP) set in the constructor.
 *  - Add function to set/query mute state from outside the class.
 *  - Add a second ring buffer for commands injected programmatically.
 *  - Add ring-buffer selection function.
 */

#include <Arduino.h>       // Corrected case (Linux FS is case-sensitive)
#include "ringBuffer.h"
#include "charAllocate.h"

// ---------------------------------------------------------------------------
// Compile-time limits
// ---------------------------------------------------------------------------

/** Maximum length (including NUL) of a single command token. */
#define MAXTOKEN    20

/** Maximum number of simultaneously registered Stream objects. */
#define MAX_STREAMS  5

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

/** Field delimiter used between command name and arguments. */
#define DELIM   ','

#define EOFch   0x1A    ///< End-of-file sentinel character
#define ACKch   0x06    ///< Positive-acknowledgement character (sent on success)
#define NAKch   0x15    ///< Negative-acknowledgement character (sent on error)

// ---------------------------------------------------------------------------
// Error codes (returned via sendNAK and stored in commandProcessor::error)
// ---------------------------------------------------------------------------

#define ERR_CMD  1  ///< Unknown command
#define ERR_ARG  2  ///< Wrong number or invalid argument(s)

// ---------------------------------------------------------------------------
// Command types
// ---------------------------------------------------------------------------

/**
 * @brief Describes how a Command entry's payload pointer is interpreted and
 *        how get/set traffic is handled.
 */
enum CmdTypes
{
    CMDstr,       ///< Pointer to a char buffer; get prints it, set copies into it
    CMDint,       ///< Pointer to an int
    CMDfloat,     ///< Pointer to a float
    CMDdouble,    ///< Pointer to a double
    CMDbool,      ///< Pointer to a bool; accepts "TRUE" / "FALSE" strings
    CMDbyte,      ///< Pointer to a uint8_t
    CMDfunction,  ///< Pointer to a void(*)(void) callback; called on match
    CMDna         ///< Placeholder / disabled entry
};

// ---------------------------------------------------------------------------
// Command descriptor
// ---------------------------------------------------------------------------

/**
 * @brief Describes a single command that the processor can match and dispatch.
 *
 * Fields
 * ------
 * cmd     — Command name string.  A leading '?' means the entry accepts both
 *           a 'G' (get) and 'S' (set) prefix at runtime.
 * type    — How `pointer` is interpreted (see CmdTypes).
 * nargs   — Expected argument count after the command token.  Pass -1 to
 *           skip the argument-count check (CMDfunction handlers typically do
 *           their own validation).
 * pointer — Typed pointer to the variable or function being exposed.
 * options — Optional constraint data.  For CMDstr: a string of valid choices.
 *           For CMDint / CMDfloat: a two-element {min, max} array.  NULL
 *           means no constraint is applied.
 * help    — Short human-readable description printed by GCMDS / HELP.
 */
typedef struct
{
    const char  *cmd;
    CmdTypes     type;
    int          nargs;    ///< Expected argument count; -1 to disable check
    void        *pointer;
    void        *options;  ///< Optional range / choice constraint (see above)
    const char  *help;
} Command;

// ---------------------------------------------------------------------------
// Command list (singly-linked chain of Command arrays)
// ---------------------------------------------------------------------------

/**
 * @brief Node in the singly-linked list of Command tables.
 *
 * Each node points to a NULL-terminated array of Command descriptors and
 * to the next CommandList node (or NULL if it is the tail).
 */
typedef struct
{
    Command *cmds;  ///< NULL-terminated array of Command entries
    void    *next;  ///< Next CommandList node, or NULL
} CommandList;

// ---------------------------------------------------------------------------
// commandProcessor class
// ---------------------------------------------------------------------------

class commandProcessor
{
    // The built-in list/help handlers need access to private members.
    friend void listCommands(void);
    friend void listCommand(void);

public:
    // -----------------------------------------------------------------------
    // Construction / destruction
    // -----------------------------------------------------------------------

    commandProcessor();
    ~commandProcessor();

    // -----------------------------------------------------------------------
    // Stream management
    // -----------------------------------------------------------------------

    /**
     * @brief Register a Stream for reading incoming command characters.
     *
     * Up to MAX_STREAMS streams may be registered.  The first registered
     * stream also becomes the default output stream (`serial`).
     *
     * @param s  Stream to add.  Silently ignored if NULL or the array is full.
     */
    void registerStream(Stream *s);

    /**
     * @brief Select a previously registered Stream as the active output.
     * @return true if @p s was found and selected; false otherwise.
     */
    bool selectStream(Stream *s);

    /**
     * @brief Return the currently selected output Stream.
     */
    Stream *selectedStream(void);

    /**
     * @brief Enable or disable processing of a particular Stream.
     *
     * While a stream is marked "do not process", its characters are not read
     * into the ring buffer even if available.
     *
     * @param s     Stream to configure.
     * @param flag  true = stop processing; false = resume processing.
     */
    void setStreamActive(Stream *s, bool active);

    // -----------------------------------------------------------------------
    // Command table management
    // -----------------------------------------------------------------------

    /**
     * @brief Append a CommandList node to the end of the registered command
     *        chain.  Ownership of the node is NOT transferred; the caller
     *        must keep it alive for the lifetime of the processor.
     */
    void registerCommands(CommandList *cmdList);

    // -----------------------------------------------------------------------
    // Processing
    // -----------------------------------------------------------------------

    /**
     * @brief Poll all active streams and copy any available characters into
     *        the internal ring buffer.  Respects the `echo` flag.
     * @return Number of streams polled (regardless of how many had data).
     */
    int processStreams(void);

    /**
     * @brief Attempt to dispatch one complete command line from the ring buffer.
     * @return true if a command line was consumed; false if the buffer holds
     *         no complete line yet.
     */
    bool processCommands(void);

    // -----------------------------------------------------------------------
    // Interactive user-input helpers
    // -----------------------------------------------------------------------

    /**
     * @brief Print @p message, then block until the user enters a line.
     * @param message   Prompt string printed before blocking.
     * @param function  Optional polling callback called while waiting.
     * @return Pointer to a charAllocate-managed buffer holding the entered
     *         text.  The caller must release it with ca->free().
     *         Returns NULL if allocation fails.
     */
    char  *userInput(const char *message, void (*function)(void) = NULL);

    /** @brief Convenience wrapper: userInput parsed as int. */
    int    userInputInt(const char *message, void (*function)(void) = NULL);

    /** @brief Convenience wrapper: userInput parsed as float. */
    float  userInputFloat(const char *message, void (*function)(void) = NULL);

    // -----------------------------------------------------------------------
    // Typed value extraction from the ring buffer
    // -----------------------------------------------------------------------

    /**
     * @brief Extract the next comma-delimited token as a string.
     *
     * @param[out] val      Receives a pointer to a charAllocate-managed buffer.
     *                      Caller must release with ca->free().
     * @param      options  If non-NULL, a comma/delimiter-separated list of
     *                      accepted values.  Returns false if token is not in
     *                      the list.
     * @return true on success; false if no token available or options check fails.
     */
    bool getValue(char **val, const char *options = NULL);

    /**
     * @brief Extract the next token as an int, optionally range-checked.
     *
     * @param[out] val  Receives the parsed integer.
     * @param ll        Lower bound (inclusive).  Pass 0 with ul==0 to skip check.
     * @param ul        Upper bound (inclusive).  Pass 0 with ll==0 to skip check.
     * @param fmt       DEC or HEX (Arduino constants).
     */
    bool getValue(int *val, int ll = 0, int ul = 0, int fmt = DEC);

    /**
     * @brief Extract the next token as a uint32_t, optionally range-checked.
     *
     * @param[out] val  Receives the parsed value.
     * @param ll        Lower bound (inclusive).  Pass 0 with ul==0 to skip check.
     * @param ul        Upper bound (inclusive).  Pass 0 with ll==0 to skip check.
     * @param fmt       DEC or HEX (Arduino constants).
     */
    bool getValue(uint32_t *val, uint32_t ll = 0, uint32_t ul = 0, int fmt = DEC);

    /**
     * @brief Extract the next token as a float, optionally range-checked.
     *
     * @param[out] val  Receives the parsed value.
     * @param ll        Lower bound (inclusive).  Pass 0.0 with ul==0.0 to skip.
     * @param ul        Upper bound (inclusive).
     */
    bool getValue(float *val, float ll = 0.0f, float ul = 0.0f);

    // -----------------------------------------------------------------------
    // Protocol output
    // -----------------------------------------------------------------------

    /**
     * @brief Send an ACK (0x06) character, optionally followed by a newline.
     * @param sendNL  If true (default), sends ACK + newline; otherwise ACK only.
     */
    void sendACK(bool sendNL = true);

    /**
     * @brief Send a NAK (0x15) + '?' sequence and record the error code.
     * @param errNum  Error code stored in `error` (default ERR_ARG).
     */
    void sendNAK(int errNum = ERR_ARG);

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /** @brief Return the most recently matched command token. */
    char *getCMD(void);

    /** @brief Return the argument count parsed for the current command. */
    int   getNumArgs(void);

    /**
     * @brief Send NAK and return false if the current argument count != @p num.
     */
    bool  checkExpectedArgs(int num);

    // -----------------------------------------------------------------------
    // Output helpers (all respect the mute flag and NULL serial guard)
    // -----------------------------------------------------------------------

    void print(void);                            ///< Print a bare newline
    void print(char *val);
    void print(const char *val);
    void print(bool val);                        ///< Prints "TRUE" or "FALSE"
    void print(int val,      int fmt = DEC);
    void print(uint32_t val, int fmt = DEC);
    void print(float val,    int fmt = 2);
    void print(double val,   int fmt = 2);
    void print(uint8_t val,  int fmt = DEC);

    /**
     * Arduino String overload. Without this, print()/println() of a String
     * matches print(bool) via String's implicit safe-bool conversion and
     * silently prints "TRUE"/"FALSE" instead of the text. The exact-match
     * reference overload takes precedence over that conversion.
     */
    void print(const String &val)  { print(val.c_str()); }

    /** println variants: print value then call print() for the newline. */
    void println(char *val)         { print(val);      print(); }
    void println(const char *val)   { print(val);      print(); }
    void println(const String &val) { print(val);      print(); }
    void println(bool val)          { print(val);      print(); }
    void println(int val,      int fmt = DEC) { print(val, fmt); print(); }
    void println(uint32_t val, int fmt = DEC) { print(val, fmt); print(); }
    void println(float val,    int fmt = 2)   { print(val, fmt); print(); }
    void println(double val,   int fmt = 2)   { print(val, fmt); print(); }
    void println(uint8_t val,  int fmt = DEC) { print(val, fmt); print(); }

    // -----------------------------------------------------------------------
    // Semi-public subsystems
    // -----------------------------------------------------------------------
    // These are intentionally accessible so that friend functions and advanced
    // callers can interact with the ring buffer and allocator directly.  In a
    // future refactor these should become proper accessor methods.

    ringBuffer   *rb;   ///< Internal ring buffer for incoming characters
    charAllocate *ca;   ///< Per-command scratch allocator
    Stream       *serial; ///< Currently active output stream

private:
    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /**
     * @brief Execute a matched Command against the current ring-buffer state.
     * @return true if the command was handled successfully; false on type or
     *         argument mismatch (caller should send NAK).
     */
    bool processCommand(Command *c);

    /**
     * @brief Search all registered CommandLists for a match against `cmd`.
     * @return Pointer to the matching Command entry, or NULL if not found.
     */
    Command *findCommand(void);

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    CommandList *commands;              ///< Head of the registered command chain
    char         cmd[MAXTOKEN];         ///< Current command token (NUL-terminated)
    bool         echo;                  ///< When true, received chars are echoed back
    bool         mute;                  ///< When true, all output is suppressed
    bool         caseSensitive;         ///< When false, commands are matched case-insensitively
    int          numArgs;               ///< Argument count for the current command line
    int          numStreams;            ///< Number of registered streams
    int          error;                 ///< Error code from the most recent sendNAK
    Stream      *streams[MAX_STREAMS];  ///< Registered input/output streams
    bool         doNotProcess[MAX_STREAMS]; ///< Per-stream processing inhibit flags
};

#endif // COMMANDPROCESSOR_H_
