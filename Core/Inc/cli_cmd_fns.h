/*! @file cli_cmd_fns.h
 * @brief Command-line interface API definitions.
 */

/*!
 * @defgroup CLI API
 */
#ifndef _CLI_CMD_FNS_H_
#define _CLI_CMD_FNS_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "cli_defs.h"
#include <stdint.h>

uint8_t cli_set_cmd_members(cli_t* cli);

// static cli_status_t help_func(cli_t cli, int argc, char **argv);
// static cli_status_t blink_func(cli_t cli, int argc, char **argv);
// static cli_status_t add_func(cli_t cli, int argc, char **argv);
// static cli_status_t multiply_func(cli_t cli, int argc, char **argv);

cli_status_t help_func(cli_t cli, int argc, char **argv);
cli_status_t blink_func(cli_t cli, int argc, char **argv);
cli_status_t add_func(cli_t cli, int argc, char **argv);
cli_status_t multiply_func(cli_t cli, int argc, char **argv);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* _CLI_CMD_FNS_H_ */
/** @}*/
