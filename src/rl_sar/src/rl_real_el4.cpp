/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_el4.hpp"
#include <cstring>

int main(int argc, char **argv)
{
    std::string gamepad_device = "/dev/input/js0";

    // Simple CLI argument parsing (no Boost dependency)
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--custom-joystick") == 0)
        {
            if (i + 1 < argc)
            {
                gamepad_device = argv[++i];
            }
            // else: no value provided, keep default "/dev/input/js0"
        }
        else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0)
        {
            std::cout << "Usage: " << argv[0] << " [OPTIONS]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --custom-joystick <device>   Gamepad device path (default: /dev/input/js0)" << std::endl;
            std::cout << "  --help, -h                   Show this help message" << std::endl;
            return 0;
        }
    }

    RL_El4 rl_el4(gamepad_device);

    // Initialize robot
    if (!rl_el4.Init())
    {
        std::cerr << "Failed to initialize El4 robot" << std::endl;
        return -1;
    }

    std::cout << "El4 robot initialized successfully" << std::endl;
    std::cout << "Gamepad device: " << gamepad_device << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;

    // Main control loop
    rl_el4.Run();

    return 0;
}
