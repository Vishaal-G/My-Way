/*
 * tts.cpp
 *
 * This file implements simple text-to-speech (TTS) functionality for the
 * our map application.
 *
 * The purpose of this file is to provide audio feedback to the
 * user when certain actions occur (e.g., selecting a street, clicking an
 * intersection, or performing a search).
 *
 * The implementation uses the external program 'spd-say' (Speech Dispatcher)
 * and launches it as a detached background process so that it does not block
 * the GTK main event loop.
 *
 * Main functionality provided:
 *   - speak(text): speaks a given string asynchronously
 *   - speak_cancel(): cancels any ongoing speech
 *
 * Internal helpers:
 *   - shell_escape(): ensures text is safely passed to the shell
 *   - log_tts(): writes debug information to /tmp/tts_debug.log
 *
 * Relationship to other files:
 *   - tts.h declares the public interface used by m2.cpp
 *   - m2.cpp calls speak() to provide audio feedback during user interaction
 *
 */

#include "tts.h"
#include <cstdlib>
#include <cstdio>
#include <string>
#include <chrono>
#include <ctime>
#include <fstream>
#include <sstream>

// Internal helper: sanitize text so it is safe to pass to the shell
static std::string shell_escape(const std::string& raw) {
    std::string out;
    out.reserve(raw.size() + 8);
    for (char c : raw) {
        if (c == '\'') {
            out += "'\\''"; // end quote, literal ', restart quote
        } else {
            out += c;
        }
    }
    return out;
}

// Internal helper: append a timestamped line to the debug log file.
static void log_tts(const std::string& text, const std::string& cmd) {
    std::ofstream log("/tmp/tts_debug.log", std::ios::app);
    if (!log.is_open()) return;

    // Human-readable timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char timebuf[32];
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    log << "[" << timebuf << "] TTS TEXT  : " << text << "\n";
    log << "[" << timebuf << "] TTS CMD   : " << cmd  << "\n";
    log << "---\n";
}

// speak()
//
// Launches spd-say in the background (non-blocking) so it never stalls the
// GTK main loop.  The "&" at the end of the shell command detaches the child
// process immediately.
//
// spd-say flags used:
//   -w          wait until the speech finishes before exiting (handled by)
//   -C          cancel any current speech first (avoid overlapping sentences)
//   -r -10      speak slightly slower than default for clarity
void speak(const std::string& text) {
    if (text.empty()) return;

    std::string safe = shell_escape(text);

    // Build the command:
    //   spd-say -C  = cancel previous utterance first
    //   spd-say -w  = wait (inside the child) so spd-say doesn't get turned off
    //   &           = run in background so we don't block GTK
    std::ostringstream cmd;
    cmd << "spd-say -C -w '" << safe << "' &";

    std::string cmd_str = cmd.str();

    // Debug log — always written regardless of audio hardware
    log_tts(text, cmd_str);

    int ret = system(cmd_str.c_str());

    // Log the return value of system() for extra debugging
    {
        std::ofstream log("/tmp/tts_debug.log", std::ios::app);
        if (log.is_open()) {
            log << "[system() return] " << ret
                << "  (0 = shell launched OK; non-zero may indicate spd-say not found)\n";
            log << "---\n";
        }
    }
}

// speak_cancel()
// Sends a cancel-speech command without speaking any new text.
// Useful if the user clicks rapidly and you want to cut off the previous speech immediately.
void speak_cancel() {
    // spd-say -C with an empty string just cancels current speech
    system("spd-say -C '' &");

    std::ofstream log("/tmp/tts_debug.log", std::ios::app);
    if (log.is_open()) {
        log << "[speak_cancel() called]\n---\n";
    }
}