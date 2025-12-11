// File: Transfer/src/Utilities/GameSystemConstants.h

#pragma once

constexpr int TARGET_FPS = 165;

const double FRAME_DELAY_MS = 1000.0 / TARGET_FPS;

const double SLOW_TIME_SCALE_FACTOR = 0.0625; // 1/16 speed

const double REGULAR_TIME_SCALE_FACTOR = 1.0f;

constexpr int FPS_UPDATE_DELTA_MS = 250;

constexpr int SCREEN_HEIGHT = 720;
constexpr int SCREEN_WIDTH = 1280;

constexpr int STAR_NUM = 200;