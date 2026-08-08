// test_main.cpp
//
// Runs every ADAPTIVE_TEST(...) registered (across all translation units
// linked into this executable) by test_framework.hpp, printing a
// pass/fail line per test and returning a nonzero exit code if any failed
// (so `ctest` / CI can detect failure from the process exit status alone).

#include "test_framework.hpp"

int main() {
    int failed = 0;
    for (const auto& test : adaptive_test::registry()) {
        try {
            test.fn();
            std::printf("[PASS] %s\n", test.name.c_str());
        } catch (const adaptive_test::TestFailure& failure) {
            std::printf("[FAIL] %s: %s\n", test.name.c_str(), failure.message.c_str());
            ++failed;
        } catch (const std::exception& error) {
            std::printf("[FAIL] %s: unexpected exception: %s\n", test.name.c_str(), error.what());
            ++failed;
        }
    }
    std::printf("%zu test(s), %d failed\n", adaptive_test::registry().size(), failed);
    return failed == 0 ? 0 : 1;
}
