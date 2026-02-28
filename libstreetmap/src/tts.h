#pragma once
#include <string>

// Speaks the given text using spd-say (non-blocking).
// Also writes to /tmp/tts_debug.log so you can verify it fired
// even when audio is unavailable (e.g. VNC sessions).
void speak(const std::string& text);

// Cancels any currently-speaking TTS immediately.
void speak_cancel();