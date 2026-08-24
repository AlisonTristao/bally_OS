#ifndef JOB_SCHEDULER_H
#define JOB_SCHEDULER_H

// autor: Alison Tristao

#include <cstddef>
#include <cstdint>

/**
 * @brief Time- and state-triggered shell commands: the robot running its own
 * policy instead of waiting to be told.
 *
 * Deliberately pure C++ -- no FreeRTOS, no esp_timer, no SD card, no
 * TinyShell. The clock arrives as a parameter to poll() and the "run this
 * line" action arrives as a SubmitFn, so the whole scheduler links into
 * env:native and is checked by test/test_job_scheduler. That matters here
 * more than usual: interval arithmetic, catch-up suppression and repeat
 * counting are exactly the kind of code that looks right and is not.
 *
 * It is also why nothing in this file may ever include TinyShell.h -- see
 * CONTRIBUTING.md. The "job" shell module lives in
 * utils/BallyRobot/BallyRobotShell.cpp.
 *
 * Ownership: schedule/cancel/list are called from the shell task, poll() once
 * per pass from the routine task. There is no lock. That is the same tolerance
 * RxRouter documents: a torn read shows a job as it was a moment earlier, the
 * job table is fixed-size and never reallocates, and every field is written as
 * a single word. Do not copy this pattern for anything larger.
 */
class JobScheduler {
public:
    /// Slots. Eight is what the boot script plus a couple of live jobs needs,
    /// and the whole table is one static ~900 octet member at that size.
    static constexpr std::size_t kMaxJobs = 8U;

    /// Longest scheduled command line. The shell accepts 512 octets, but a
    /// job is meant to be one short command, and this bounds the table.
    static constexpr std::size_t kMaxCommandLength = 96U;

    enum class Kind : std::uint8_t {
        Interval,      ///< every N ms, `remaining` times (0 = forever)
        Once,          ///< a single time, N ms from scheduling
        OnStateEnter   ///< every time the machine ENTERS a given state
    };

    /**
     * @brief Hands one command line to whoever executes it.
     *
     * Returns false when the line could not be accepted right now (a full
     * command queue is the real case). A job that cannot be delivered is
     * counted and dropped, never retried -- see poll().
     */
    using SubmitFn = bool (*)(void* context, const char* command_line);

    /// Why a schedule call was refused. One enumerator per cause so the shell
    /// can say which, instead of a bare "failed".
    enum class Error : std::uint8_t {
        Ok = 0,
        Full,            ///< every slot is taken
        EmptyCommand,    ///< no command text
        CommandTooLong,  ///< longer than kMaxCommandLength
        BadInterval,     ///< zero interval/delay
        Recursive,       ///< the command is itself a "job" command
        NotConfigured    ///< configure() was never called
    };

    /// Read-only snapshot of one job, for "job -list".
    struct JobView {
        std::uint8_t  id;
        Kind          kind;
        std::uint32_t interval_ms;  ///< 0 for OnStateEnter
        std::uint8_t  state;        ///< OnStateEnter only
        std::uint32_t remaining;    ///< 0 means "no limit"
        std::uint32_t fired;
        const char*   command;
    };

    struct Stats {
        std::uint32_t fired = 0U;          ///< commands handed to SubmitFn
        std::uint32_t submit_failed = 0U;  ///< SubmitFn said no (queue full)
        std::uint32_t rejected = 0U;       ///< schedule calls refused
        std::uint32_t skipped_late = 0U;   ///< occurrences dropped, see poll()
    };

    /**
     * @brief Bind the executor. Until this is called every schedule attempt is
     * refused with NotConfigured -- a job table nobody can run is worse than
     * an error at the moment of scheduling.
     */
    void configure(SubmitFn submit, void* context) noexcept;

    /**
     * @brief Run `command` every `interval_ms`.
     * @param repeat How many times; 0 means until cancelled.
     * @param id_out Receives the new job's id when the call succeeds.
     */
    Error schedule_interval(std::uint32_t interval_ms, std::uint32_t repeat,
                            const char* command, std::uint8_t* id_out) noexcept;

    /** @brief Run `command` once, `delay_ms` from now. */
    Error schedule_once(std::uint32_t delay_ms, const char* command,
                        std::uint8_t* id_out) noexcept;

    /**
     * @brief Run `command` every time the machine ENTERS `state`.
     *
     * Entering, not being in: poll() compares the state it is given against
     * the one from the previous call, so a job attached to a state the robot
     * sits in for a minute fires once, not sixty thousand times.
     *
     * `state` is an opaque number here on purpose -- this library does not
     * know stateName. The shell command translates the name.
     */
    Error schedule_on_state(std::uint8_t state, const char* command,
                            std::uint8_t* id_out) noexcept;

    /** @brief @return false when no active job carries that id. */
    bool cancel(std::uint8_t id) noexcept;

    /** @brief @return how many jobs were cancelled. */
    std::size_t cancel_all() noexcept;

    std::size_t list(JobView* out, std::size_t max_count) const noexcept;
    std::size_t active_count() const noexcept;

    /**
     * @brief Fire whatever is due. Call once per pass from a single task.
     *
     * Two rules worth knowing before scheduling anything:
     *
     * 1. **A job is a schedule, not a delivery guarantee.** If SubmitFn
     *    refuses (command queue full), the occurrence is counted in
     *    submit_failed and consumed anyway. It is not retried, and it is not
     *    saved up.
     * 2. **No catch-up bursts.** If this is called late by more than one
     *    whole interval -- a long OTA write, USB owning the card -- the
     *    missed occurrences are counted in skipped_late and dropped, and the
     *    schedule resyncs from now. A robot that stalls for ten seconds must
     *    not then fire ten seconds of backlog into the motors at once.
     *
     * @param now_ms       Milliseconds since boot; must not go backwards.
     * @param current_state The value OnStateEnter jobs are matched against.
     * @return how many commands were handed to SubmitFn.
     */
    std::size_t poll(std::uint64_t now_ms, std::uint8_t current_state) noexcept;

    Stats stats() const noexcept { return stats_; }
    void reset_stats() noexcept { stats_ = Stats{}; }

    static const char* error_text(Error error) noexcept;
    static const char* kind_text(Kind kind) noexcept;

    /**
     * @brief Whether `command_line`'s first word is this module's own name.
     *
     * Public because the script runner needs the same check: a script line or
     * a job that schedules jobs is a self-replicating loop with no bound, and
     * eight slots fill in eight passes. Refused at the door instead.
     */
    static bool is_job_command(const char* command_line) noexcept;

private:
    struct Job {
        bool          active = false;
        Kind          kind = Kind::Interval;
        std::uint8_t  id = 0U;
        std::uint8_t  state = 0U;
        std::uint32_t interval_ms = 0U;
        std::uint64_t next_ms = 0U;
        std::uint32_t remaining = 0U;  // 0 = unlimited (Interval only)
        std::uint32_t fired = 0U;
        char          command[kMaxCommandLength + 1U] = {};
    };

    /// Common validation + slot allocation for the three schedule_* calls.
    Error allocate(const char* command, Job** slot_out) noexcept;

    Job          jobs_[kMaxJobs]{};
    SubmitFn     submit_ = nullptr;
    void*        submit_context_ = nullptr;
    Stats        stats_{};

    // Ids are never reused within a boot, so "job -cancel 3" can't hit a
    // different job than the one the operator saw in "job -list". Wraps past
    // 255 back to 1; 0 is reserved for "no id".
    std::uint8_t next_id_ = 1U;

    // Previous value seen by poll(), for OnStateEnter edge detection.
    // kNoState means "poll() has not run yet", so the very first call does not
    // read the initial state as an entry into it.
    static constexpr std::uint8_t kNoState = 0xFFU;
    std::uint8_t last_state_ = kNoState;
};

#endif  // JOB_SCHEDULER_H
