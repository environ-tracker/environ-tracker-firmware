/*
 * Copyright (c) 2022 Vestas Wind Systems A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_SEMTECH_SX1508B_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_SEMTECH_SX1508B_H_

/**
 * @brief Enable GPIO pin debounce.
 *
 * The debounce flag is a Zephyr specific extension of the standard GPIO flags
 * specified by the Linux GPIO binding. Only applicable for Semtech SX1508B GPIO
 * controllers.
 */
#define SX1508B_GPIO_DEBOUNCE (1U << 8)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_SEMTECH_SX1508B_H_ */
