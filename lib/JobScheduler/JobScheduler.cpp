#include <JobScheduler.h>

#include <cstring>

namespace {

// Skips leading blanks and reports whether what follows is the word "job".
// Matches the shell's own parse: a command line is "module -command args",
// so the module name ends at the first space (or at the '-' when someone
// writes "job-list" with no space).
bool first_word_is_job(const char* text) {
    while (*text == ' ' || *text == '\t') ++text;

    static const char kName[] = "job";
    for (std::size_t i = 0; i < sizeof(kName) - 1U; ++i) {
        char c = text[i];
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c + ('a' - 'A'));
        if (c != kName[i]) return false;
    }

    const char after = text[sizeof(kName) - 1U];
    return after == '\0' || after == ' ' || after == '\t' || after == '-';
}

}  // namespace

bool JobScheduler::is_job_command(const char* command_line) noexcept {
    if (command_line == nullptr) return false;
    return first_word_is_job(command_line);
}

void JobScheduler::configure(SubmitFn submit, void* context) noexcept {
    submit_ = submit;
    submit_context_ = context;
}

JobScheduler::Error JobScheduler::allocate(const char* command,
                                           Job** slot_out) noexcept {
    if (submit_ == nullptr) {
        ++stats_.rejected;
        return Error::NotConfigured;
    }
    if (command == nullptr || command[0] == '\0') {
        ++stats_.rejected;
        return Error::EmptyCommand;
    }

    const std::size_t length = std::strlen(command);
    if (length > kMaxCommandLength) {
        ++stats_.rejected;
        return Error::CommandTooLong;
    }
    if (is_job_command(command)) {
        // A job that schedules jobs fills every slot within a few passes and
        // there is no sensible bound to put on it. Refuse at the door.
        ++stats_.rejected;
        return Error::Recursive;
    }

    for (std::size_t i = 0; i < kMaxJobs; ++i) {
        if (jobs_[i].active) continue;

        Job& job = jobs_[i];
        job = Job{};
        job.active = true;
        job.id = next_id_;
        std::memcpy(job.command, command, length);
        job.command[length] = '\0';

        next_id_ = (next_id_ == 0xFFU) ? 1U : static_cast<std::uint8_t>(next_id_ + 1U);

        *slot_out = &job;
        return Error::Ok;
    }

    ++stats_.rejected;
    return Error::Full;
}

JobScheduler::Error JobScheduler::schedule_interval(std::uint32_t interval_ms,
                                                    std::uint32_t repeat,
                                                    const char* command,
                                                    std::uint8_t* id_out) noexcept {
    if (interval_ms == 0U) {
        ++stats_.rejected;
        return Error::BadInterval;
    }

    Job* job = nullptr;
    const Error error = allocate(command, &job);
    if (error != Error::Ok) return error;

    job->kind = Kind::Interval;
    job->interval_ms = interval_ms;
    job->remaining = repeat;  // 0 stays 0: unlimited
    // next_ms is left at 0 and fixed up on the first poll(), because this call
    // has no clock. The first firing therefore lands one full interval after
    // the next poll(), not at an arbitrary distance from boot.
    if (id_out != nullptr) *id_out = job->id;
    return Error::Ok;
}

JobScheduler::Error JobScheduler::schedule_once(std::uint32_t delay_ms,
                                                const char* command,
                                                std::uint8_t* id_out) noexcept {
    if (delay_ms == 0U) {
        ++stats_.rejected;
        return Error::BadInterval;
    }

    Job* job = nullptr;
    const Error error = allocate(command, &job);
    if (error != Error::Ok) return error;

    job->kind = Kind::Once;
    job->interval_ms = delay_ms;
    job->remaining = 1U;
    if (id_out != nullptr) *id_out = job->id;
    return Error::Ok;
}

JobScheduler::Error JobScheduler::schedule_on_state(std::uint8_t state,
                                                    const char* command,
                                                    std::uint8_t* id_out) noexcept {
    Job* job = nullptr;
    const Error error = allocate(command, &job);
    if (error != Error::Ok) return error;

    job->kind = Kind::OnStateEnter;
    job->state = state;
    job->interval_ms = 0U;
    job->remaining = 0U;  // no limit: it fires on every entry
    if (id_out != nullptr) *id_out = job->id;
    return Error::Ok;
}

bool JobScheduler::cancel(std::uint8_t id) noexcept {
    for (std::size_t i = 0; i < kMaxJobs; ++i) {
        if (jobs_[i].active && jobs_[i].id == id) {
            jobs_[i].active = false;
            return true;
        }
    }
    return false;
}

std::size_t JobScheduler::cancel_all() noexcept {
    std::size_t cancelled = 0;
    for (std::size_t i = 0; i < kMaxJobs; ++i) {
        if (jobs_[i].active) {
            jobs_[i].active = false;
            ++cancelled;
        }
    }
    return cancelled;
}

std::size_t JobScheduler::list(JobView* out, std::size_t max_count) const noexcept {
    if (out == nullptr || max_count == 0U) return 0;

    std::size_t written = 0;
    for (std::size_t i = 0; i < kMaxJobs && written < max_count; ++i) {
        const Job& job = jobs_[i];
        if (!job.active) continue;

        out[written].id = job.id;
        out[written].kind = job.kind;
        out[written].interval_ms = job.interval_ms;
        out[written].state = job.state;
        out[written].remaining = job.remaining;
        out[written].fired = job.fired;
        out[written].command = job.command;
        ++written;
    }
    return written;
}

std::size_t JobScheduler::active_count() const noexcept {
    std::size_t count = 0;
    for (std::size_t i = 0; i < kMaxJobs; ++i) {
        if (jobs_[i].active) ++count;
    }
    return count;
}

std::size_t JobScheduler::poll(std::uint64_t now_ms,
                               std::uint8_t current_state) noexcept {
    if (submit_ == nullptr) return 0;

    // OnStateEnter is an edge, not a level: compare against the previous call.
    // The first ever poll() only records the state -- otherwise every job
    // attached to the state the robot happens to boot into would fire once at
    // startup, which is not what "on entering" means.
    const bool state_entered =
        (last_state_ != kNoState) && (current_state != last_state_);
    last_state_ = current_state;

    std::size_t fired_now = 0;

    for (std::size_t i = 0; i < kMaxJobs; ++i) {
        Job& job = jobs_[i];
        if (!job.active) continue;

        bool due = false;

        if (job.kind == Kind::OnStateEnter) {
            due = state_entered && (job.state == current_state);
        } else {
            if (job.next_ms == 0U) {
                // First poll() after scheduling: anchor the schedule to the
                // clock we finally have.
                job.next_ms = now_ms + job.interval_ms;
                continue;
            }
            due = (now_ms >= job.next_ms);
        }

        if (!due) continue;

        if (job.kind != Kind::OnStateEnter) {
            // Drop whole missed occurrences instead of firing them back to
            // back. One interval of lateness is normal jitter; more than that
            // means something blocked the task, and replaying that backlog
            // into actuators is the failure this avoids.
            const std::uint64_t lateness = now_ms - job.next_ms;
            if (lateness >= job.interval_ms) {
                const std::uint64_t missed = lateness / job.interval_ms;
                stats_.skipped_late += static_cast<std::uint32_t>(missed);
                job.next_ms = now_ms + job.interval_ms;
            } else {
                job.next_ms += job.interval_ms;
            }
        }

        if (submit_(submit_context_, job.command)) {
            ++stats_.fired;
            ++fired_now;
        } else {
            // Counted and dropped, never retried: see the poll() contract.
            ++stats_.submit_failed;
        }
        ++job.fired;

        if (job.remaining != 0U) {
            --job.remaining;
            if (job.remaining == 0U) job.active = false;
        }
    }

    return fired_now;
}

const char* JobScheduler::error_text(Error error) noexcept {
    switch (error) {
        case Error::Ok:             return "ok";
        case Error::Full:           return "no free job slot";
        case Error::EmptyCommand:   return "empty command";
        case Error::CommandTooLong: return "command longer than 96 characters";
        case Error::BadInterval:    return "interval must be greater than zero";
        case Error::Recursive:      return "a job may not schedule jobs";
        case Error::NotConfigured:  return "scheduler has no executor";
    }
    return "unknown error";
}

const char* JobScheduler::kind_text(Kind kind) noexcept {
    switch (kind) {
        case Kind::Interval:     return "every";
        case Kind::Once:         return "once";
        case Kind::OnStateEnter: return "at";
    }
    return "unknown";
}
