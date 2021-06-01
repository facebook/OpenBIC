/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _GETOPT_H
#define _GETOPT_H 1

extern char *optarg;
extern int optind;
extern int opterr;
extern int optopt;

extern int getopt (int argc, char *const *argv, const char *shortopts);

#endif /* _GETOPT_H */
