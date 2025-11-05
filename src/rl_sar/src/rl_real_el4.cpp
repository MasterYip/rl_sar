/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_el4.hpp"

int main(int argc, char **argv)
{
    RL_El4 rl_el4;

    // Initialize robot
    if (!rl_el4.Init())
    {
        std::cerr << "Failed to initialize El4 robot" << std::endl;
        return -1;
    }

    std::cout << "El4 robot initialized successfully" << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;

    // Main control loop
    rl_el4.Run();

    return 0;
}
