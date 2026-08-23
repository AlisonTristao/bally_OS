#include <unity.h>

#include <Format.h>

#include <cstring>

// formatBytes() is shared by every module that reports storage/memory usage
// over the shell (SDCard, USBMassStorage, Logger) specifically so the unit
// picked and the rounding shown are identical everywhere -- this suite pins
// exactly that: the unit boundaries (B/kB/MB/GB switch at 1024, not before
// or after) and that a caller-supplied buffer that is too small is truncated
// safely rather than overflowed.

namespace {

void assert_formats_to(std::uint64_t bytes, const char* expected) {
    char output[32];
    formatBytes(bytes, output, sizeof(output));
    TEST_ASSERT_EQUAL_STRING(expected, output);
}

void test_zero_bytes() {
    assert_formats_to(0U, "0.00 B");
}

void test_stays_in_bytes_up_to_the_kb_boundary() {
    assert_formats_to(500U, "500.00 B");
    // One octet short of the kB threshold: still reported in bytes, not
    // "1.00 kB" -- the switch happens strictly at 1024, not somewhere close
    // to it.
    assert_formats_to(1023U, "1023.00 B");
}

void test_switches_to_kilobytes_at_1024() {
    assert_formats_to(1024U, "1.00 kB");
    assert_formats_to(2048U, "2.00 kB");
    assert_formats_to(10240U, "10.00 kB");
}

void test_stays_in_kilobytes_up_to_the_mb_boundary() {
    assert_formats_to((1024ULL * 1024ULL) - 1024ULL, "1023.00 kB");
}

void test_switches_to_megabytes_at_1024_squared() {
    assert_formats_to(1024ULL * 1024ULL, "1.00 MB");
    assert_formats_to(2ULL * 1024ULL * 1024ULL, "2.00 MB");
}

void test_stays_in_megabytes_up_to_the_gb_boundary() {
    assert_formats_to((1024ULL * 1024ULL * 1024ULL) - (1024ULL * 1024ULL),
                      "1023.00 MB");
}

void test_switches_to_gigabytes_at_1024_cubed() {
    assert_formats_to(1024ULL * 1024ULL * 1024ULL, "1.00 GB");
    assert_formats_to(5ULL * 1024ULL * 1024ULL * 1024ULL, "5.00 GB");
}

// GB is the top unit -- there is no TB tier, so even a very large uint64_t
// value must still land on "GB" rather than overflow into a bogus unit.
void test_very_large_values_stay_in_gigabytes() {
    assert_formats_to(4000ULL * 1024ULL * 1024ULL * 1024ULL, "4000.00 GB");
}

// A caller-supplied buffer smaller than the formatted result (e.g. a
// terminal column width) must truncate safely -- still NUL-terminated
// within capacity, never written past it. This is what snprintf's return
// contract gives formatBytes for free, but it is exactly the kind of thing a
// future rewrite (e.g. swapping snprintf for a hand-rolled formatter) could
// quietly break.
void test_truncates_safely_when_the_buffer_is_too_small() {
    char output[6] = {};
    formatBytes(5ULL * 1024ULL * 1024ULL * 1024ULL, output, sizeof(output));
    TEST_ASSERT_EQUAL_UINT32(sizeof(output) - 1U, strlen(output));
    TEST_ASSERT_EQUAL_STRING("5.00 ", output);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_zero_bytes);
    RUN_TEST(test_stays_in_bytes_up_to_the_kb_boundary);
    RUN_TEST(test_switches_to_kilobytes_at_1024);
    RUN_TEST(test_stays_in_kilobytes_up_to_the_mb_boundary);
    RUN_TEST(test_switches_to_megabytes_at_1024_squared);
    RUN_TEST(test_stays_in_megabytes_up_to_the_gb_boundary);
    RUN_TEST(test_switches_to_gigabytes_at_1024_cubed);
    RUN_TEST(test_very_large_values_stay_in_gigabytes);
    RUN_TEST(test_truncates_safely_when_the_buffer_is_too_small);
    return UNITY_END();
}
