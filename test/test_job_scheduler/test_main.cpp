#include <unity.h>

#include <JobScheduler.h>

#include <cstring>
#include <string>
#include <vector>

// JobScheduler is pure C++ precisely so this suite can exist: interval
// arithmetic, catch-up suppression, repeat counting and edge detection are
// the kind of code that looks obviously right and is not. None of it is
// reachable from the target build without a robot on the bench.

namespace {

// Records every line the scheduler hands out, and can refuse on demand to
// stand in for a full command queue.
struct Executor {
    std::vector<std::string> submitted;
    bool accept = true;

    static bool submit(void* context, const char* line) {
        Executor* self = static_cast<Executor*>(context);
        if (!self->accept) return false;
        self->submitted.emplace_back(line);
        return true;
    }
};

constexpr std::uint8_t kStateWait = 2U;
constexpr std::uint8_t kStateRun  = 5U;

}  // namespace

void setUp(void) {}
void tearDown(void) {}

static void test_unconfigured_scheduler_refuses_to_schedule(void) {
    JobScheduler scheduler;
    std::uint8_t id = 0;

    TEST_ASSERT_EQUAL(JobScheduler::Error::NotConfigured,
                      scheduler.schedule_interval(100U, 0U, "sys -uptime", &id));
    TEST_ASSERT_EQUAL_UINT32(1U, scheduler.stats().rejected);
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.active_count());
}

static void test_interval_job_fires_on_the_interval_not_before(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    std::uint8_t id = 0;
    TEST_ASSERT_EQUAL(JobScheduler::Error::Ok,
                      scheduler.schedule_interval(1000U, 0U, "sys -uptime", &id));
    TEST_ASSERT_NOT_EQUAL(0U, id);

    // First poll only anchors the schedule to the clock; nothing fires yet.
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.poll(5000U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    TEST_ASSERT_EQUAL_size_t(0U, scheduler.poll(5999U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(1U, scheduler.poll(6000U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());
    TEST_ASSERT_EQUAL_STRING("sys -uptime", exec.submitted[0].c_str());

    TEST_ASSERT_EQUAL_size_t(1U, scheduler.poll(7000U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(2U, exec.submitted.size());
}

static void test_repeat_count_deactivates_the_job_when_exhausted(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    std::uint8_t id = 0;
    scheduler.schedule_interval(100U, 3U, "sensor -position", &id);

    scheduler.poll(0U, kStateWait);  // anchor
    for (int i = 1; i <= 5; ++i) {
        scheduler.poll(static_cast<std::uint64_t>(i) * 100U, kStateWait);
    }

    TEST_ASSERT_EQUAL_size_t(3U, exec.submitted.size());
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.active_count());
}

static void test_once_job_fires_a_single_time_then_frees_its_slot(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    scheduler.schedule_once(250U, "junkebox -play_builtin boot", nullptr);

    scheduler.poll(1000U, kStateWait);  // anchor
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    scheduler.poll(1250U, kStateWait);
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.active_count());

    scheduler.poll(5000U, kStateWait);
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());
}

static void test_state_job_fires_on_entry_not_while_resident(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    scheduler.schedule_on_state(kStateWait, "sensor -calibrate", nullptr);

    // The very first poll only records the state: booting into WAIT is not an
    // entry into WAIT as far as an "on entering" job is concerned.
    scheduler.poll(0U, kStateWait);
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    // Sitting in WAIT must not fire it over and over.
    scheduler.poll(10U, kStateWait);
    scheduler.poll(20U, kStateWait);
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    scheduler.poll(30U, kStateRun);   // leave
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    scheduler.poll(40U, kStateWait);  // re-enter -> fires
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());

    scheduler.poll(50U, kStateWait);  // still resident -> silent
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());

    scheduler.poll(60U, kStateRun);
    scheduler.poll(70U, kStateWait);  // enters again -> fires again
    TEST_ASSERT_EQUAL_size_t(2U, exec.submitted.size());
}

static void test_state_job_ignores_a_different_state(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    scheduler.schedule_on_state(kStateRun, "kalman -state", nullptr);

    scheduler.poll(0U, kStateWait);
    scheduler.poll(10U, kStateWait);
    scheduler.poll(20U, kStateWait);
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    scheduler.poll(30U, kStateRun);
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());
}

static void test_long_stall_does_not_replay_a_backlog(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    scheduler.schedule_interval(100U, 0U, "robot -set_pwm_pair 30\\, 30\\, 200",
                                nullptr);
    scheduler.poll(0U, kStateWait);  // anchor: next at 100

    // The task was blocked for ten whole seconds (an OTA write, USB owning the
    // card). One hundred occurrences came due. Exactly one fires.
    TEST_ASSERT_EQUAL_size_t(1U, scheduler.poll(10000U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());
    TEST_ASSERT_EQUAL_UINT32(99U, scheduler.stats().skipped_late);

    // And the schedule resyncs from now rather than staying behind.
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.poll(10050U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(1U, scheduler.poll(10100U, kStateWait));
}

static void test_refused_submission_is_counted_and_not_retried(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    scheduler.schedule_interval(100U, 0U, "sys -uptime", nullptr);
    scheduler.poll(0U, kStateWait);  // anchor

    exec.accept = false;
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.poll(100U, kStateWait));
    TEST_ASSERT_EQUAL_UINT32(1U, scheduler.stats().submit_failed);
    TEST_ASSERT_EQUAL_size_t(0U, exec.submitted.size());

    // The refused occurrence is gone, not saved up: the next poll at the next
    // interval fires once, not twice.
    exec.accept = true;
    TEST_ASSERT_EQUAL_size_t(1U, scheduler.poll(200U, kStateWait));
    TEST_ASSERT_EQUAL_size_t(1U, exec.submitted.size());
}

static void test_a_job_may_not_schedule_jobs(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    TEST_ASSERT_EQUAL(JobScheduler::Error::Recursive,
                      scheduler.schedule_interval(100U, 0U, "job -list", nullptr));
    TEST_ASSERT_EQUAL(JobScheduler::Error::Recursive,
                      scheduler.schedule_once(100U, "  JOB -cancel_all", nullptr));
    TEST_ASSERT_EQUAL(JobScheduler::Error::Recursive,
                      scheduler.schedule_on_state(kStateRun, "job-list", nullptr));
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.active_count());

    // A command that merely starts with the same letters is fine.
    TEST_ASSERT_EQUAL(JobScheduler::Error::Ok,
                      scheduler.schedule_once(100U, "jobsite -status", nullptr));
}

static void test_validation_reports_the_specific_cause(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    TEST_ASSERT_EQUAL(JobScheduler::Error::BadInterval,
                      scheduler.schedule_interval(0U, 0U, "sys -uptime", nullptr));
    TEST_ASSERT_EQUAL(JobScheduler::Error::EmptyCommand,
                      scheduler.schedule_interval(100U, 0U, "", nullptr));
    TEST_ASSERT_EQUAL(JobScheduler::Error::EmptyCommand,
                      scheduler.schedule_interval(100U, 0U, nullptr, nullptr));

    const std::string too_long(JobScheduler::kMaxCommandLength + 1U, 'x');
    TEST_ASSERT_EQUAL(JobScheduler::Error::CommandTooLong,
                      scheduler.schedule_interval(100U, 0U, too_long.c_str(),
                                                  nullptr));

    // Exactly at the limit is accepted.
    const std::string at_limit(JobScheduler::kMaxCommandLength, 'x');
    TEST_ASSERT_EQUAL(JobScheduler::Error::Ok,
                      scheduler.schedule_interval(100U, 0U, at_limit.c_str(),
                                                  nullptr));
}

static void test_table_fills_and_refuses_the_ninth_job(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    for (std::size_t i = 0; i < JobScheduler::kMaxJobs; ++i) {
        TEST_ASSERT_EQUAL(JobScheduler::Error::Ok,
                          scheduler.schedule_interval(100U, 0U, "sys -uptime",
                                                      nullptr));
    }
    TEST_ASSERT_EQUAL(JobScheduler::Error::Full,
                      scheduler.schedule_interval(100U, 0U, "sys -uptime",
                                                  nullptr));
    TEST_ASSERT_EQUAL_size_t(JobScheduler::kMaxJobs, scheduler.active_count());
}

static void test_cancel_frees_the_slot_and_ids_are_not_reused(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    std::uint8_t first = 0;
    std::uint8_t second = 0;
    scheduler.schedule_interval(100U, 0U, "sys -uptime", &first);
    TEST_ASSERT_TRUE(scheduler.cancel(first));
    TEST_ASSERT_FALSE(scheduler.cancel(first));

    scheduler.schedule_interval(100U, 0U, "sys -temp", &second);
    // Reusing the id would let "job -cancel <n>" hit a different job than the
    // one the operator saw listed.
    TEST_ASSERT_NOT_EQUAL(first, second);
    TEST_ASSERT_EQUAL_size_t(1U, scheduler.active_count());

    TEST_ASSERT_EQUAL_size_t(1U, scheduler.cancel_all());
    TEST_ASSERT_EQUAL_size_t(0U, scheduler.active_count());
}

static void test_list_reports_what_was_scheduled(void) {
    Executor exec;
    JobScheduler scheduler;
    scheduler.configure(&Executor::submit, &exec);

    std::uint8_t interval_id = 0;
    std::uint8_t state_id = 0;
    scheduler.schedule_interval(2500U, 4U, "sensor -position", &interval_id);
    scheduler.schedule_on_state(kStateWait, "sensor -calibrate", &state_id);

    JobScheduler::JobView views[JobScheduler::kMaxJobs];
    TEST_ASSERT_EQUAL_size_t(2U, scheduler.list(views, JobScheduler::kMaxJobs));

    TEST_ASSERT_EQUAL_UINT8(interval_id, views[0].id);
    TEST_ASSERT_EQUAL(JobScheduler::Kind::Interval, views[0].kind);
    TEST_ASSERT_EQUAL_UINT32(2500U, views[0].interval_ms);
    TEST_ASSERT_EQUAL_UINT32(4U, views[0].remaining);
    TEST_ASSERT_EQUAL_STRING("sensor -position", views[0].command);

    TEST_ASSERT_EQUAL_UINT8(state_id, views[1].id);
    TEST_ASSERT_EQUAL(JobScheduler::Kind::OnStateEnter, views[1].kind);
    TEST_ASSERT_EQUAL_UINT8(kStateWait, views[1].state);
    TEST_ASSERT_EQUAL_STRING("sensor -calibrate", views[1].command);

    // A caller with room for one gets one, not a buffer overrun.
    TEST_ASSERT_EQUAL_size_t(1U, scheduler.list(views, 1U));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_unconfigured_scheduler_refuses_to_schedule);
    RUN_TEST(test_interval_job_fires_on_the_interval_not_before);
    RUN_TEST(test_repeat_count_deactivates_the_job_when_exhausted);
    RUN_TEST(test_once_job_fires_a_single_time_then_frees_its_slot);
    RUN_TEST(test_state_job_fires_on_entry_not_while_resident);
    RUN_TEST(test_state_job_ignores_a_different_state);
    RUN_TEST(test_long_stall_does_not_replay_a_backlog);
    RUN_TEST(test_refused_submission_is_counted_and_not_retried);
    RUN_TEST(test_a_job_may_not_schedule_jobs);
    RUN_TEST(test_validation_reports_the_specific_cause);
    RUN_TEST(test_table_fills_and_refuses_the_ninth_job);
    RUN_TEST(test_cancel_frees_the_slot_and_ids_are_not_reused);
    RUN_TEST(test_list_reports_what_was_scheduled);
    return UNITY_END();
}
