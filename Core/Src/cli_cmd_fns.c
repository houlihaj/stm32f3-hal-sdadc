/*! @file cli_cmd_fns.c
 * @brief Implementation of command-line interface.
 */
#include "cli_cmd_fns.h"
#include "printf.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

cmd_t cmd_tbl[] = {
    {.cmd = "help", .func = help_func},
    {.cmd = "blink", .func = blink_func},
    {.cmd = "add", .func = add_func},
    {.cmd = "multiply", .func = multiply_func}
};

/**
  * @brief  asdf
  * @note   To use in conjunction with cli.c library
  * @param  asdf
  * @retval uint8_t
  */
uint8_t cli_set_cmd_members(cli_t* cli) {
    cli->cmd_tbl = cmd_tbl;
    cli->cmd_cnt = sizeof(cmd_tbl) / sizeof(cmd_t);
    return 0;
}

/**
  * @brief  asdf
  * @note   To use in conjunction with cli.c library
  * @param  asdf
  * @retval cli_status_t
  */
cli_status_t help_func(cli_t cli, int argc, char **argv)
{
    cli.println("HELP function executed\r\n");
    return CLI_OK;
}

/**
  * @brief  asdf
  * @note   To use in conjunction with cli.c library
  * @param  asdf
  * @retval cli_status_t
  */
cli_status_t blink_func(cli_t cli, int argc, char **argv)
{
    if (argc > 0) {
        if (strcmp(argv[1], "-H") == 0 || strcmp(argv[1], "--help") == 0) {
            cli.println("BLINK help menu\r\n");
        } else {
            return CLI_E_INVALID_ARGS;
        }
    } else {
        cli.println("BLINK function executed\r\n");
    }
    return CLI_OK;
}

/**
  * @brief  asdf
  * @note   To use in conjunction with cli.c library
  * @param  asdf
  * @retval cli_status_t
  */
cli_status_t add_func(cli_t cli, int argc, char **argv)
{
    cli.println("ADD function executed\r\n");
    if (argc > 2) {
        size_t len = 32;
        char String[len];
        uint8_t a = atoi(argv[1]);
        uint8_t b = atoi(argv[2]);
        uint16_t c = a + b;
        snprintf_(String, len, "%d + %d = %d\r\n", a, b, c);
        cli.println(String);
    }
    return CLI_OK;
}

/**
  * @brief  asdf
  * @note   To use in conjunction with cli.c library
  * @param  asdf
  * @retval cli_status_t
  */
cli_status_t multiply_func(cli_t cli, int argc, char **argv)
{
    cli.println("MULTIPLY function executed\r\n");
    if (argc > 2) {
        size_t len = 32;
        char String[len];
        uint8_t a = atoi(argv[1]);
        uint8_t b = atoi(argv[2]);
        uint16_t c = a * b;
        snprintf_(String, len, "%d * %d = %d\r\n", a, b, c);
        cli.println(String);
    }
    return CLI_OK;
}
