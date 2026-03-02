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

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>

#include "tts.h"

// Escape single quotes so the text is safe to pass to a shell command
static std::string shell_escape(const std::string& raw) {
  std::string out;
  out.reserve(raw.size() + 8);
  for (char c : raw) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out += c;
    }
  }
  return out;
}

// Write the TTS text and command to a log file for debugging
static void log_tts(const std::string& text, const std::string& cmd) {
  std::ofstream log("/tmp/tts_debug.log", std::ios::app);
  if (!log.is_open()) return;

  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  char timebuf[32];
  std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S",
                std::localtime(&t));

  log << "[" << timebuf << "] TTS TEXT  : " << text << "\n";
  log << "[" << timebuf << "] TTS CMD   : " << cmd << "\n";
  log << "---\n";
}

// Speak the given text out loud using spd-say, cancelling any ongoing speech
// first
void speak(const std::string& text) {
  if (text.empty()) return;

  std::string safe = shell_escape(text);

  std::ostringstream cmd;
  cmd << "spd-say -C -w '" << safe << "' &";

  std::string cmd_str = cmd.str();

  log_tts(text, cmd_str);

  int ret = system(cmd_str.c_str());

  {
    std::ofstream log("/tmp/tts_debug.log", std::ios::app);
    if (log.is_open()) {
      log << "[system() return] " << ret
          << "  (0 = shell launched OK; non-zero may indicate spd-say not "
             "found)\n";
      log << "---\n";
    }
  }
}

// Cancel any speech currently playing
void speak_cancel() {
  system("spd-say -C '' &");

  // Log the cancellation for debugging
  std::ofstream log("/tmp/tts_debug.log", std::ios::app);
  if (log.is_open()) {
    log << "[speak_cancel() called]\n---\n";
  }
}