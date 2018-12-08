/*-
 * Copyright (c) 2019 Netflix, Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

#ifndef	ARG_H
#define	ARG_H

#include <sys/linker_set.h>

/*
 * Regularized parsing of simple arguments built on top of getopt_long.
 */

typedef enum arg_type {
	arg_none = 0,
	arg_uint8,
	arg_uint16,
	arg_uint32,
	arg_string,
	arg_path,
} arg_type;

struct opts {
	const char	*long_arg;
	int		short_arg;
	arg_type	at;
	void 		*ptr;
	const char	*descr;
};

typedef void (*cmd_load_cb_t)(void *, void *);
struct cmd_function;
typedef void (*cmd_fn_t)(const struct cmd_function *nf, int argc, char *argv[]);

struct cmd_function {
	const char	*name;
	cmd_fn_t	fn;
	const char	*usage;
	const char	*args;
	const char	*descr;
};

#define CMD_SETNAME(set)	set
#define	CMD_COMMAND_SET(set, sym)	DATA_SET(CMD_SETNAME(set), sym)
#define CMD_COMMAND(set, nam, function, usage_str, args_str, descr_str) \
	static struct cmd_function function ## _cmd_command =		\
	{ .name = #nam, .fn = function, .usage = usage_str,		\
	  .args = args_str, .descr = descr_str };			\
	CMD_COMMAND_SET(set, function ## _cmd_command)
#define CMD_BEGIN(set)	SET_BEGIN(CMD_SETNAME(set))
#define CMD_LIMIT(set)	SET_LIMIT(CMD_SETNAME(set))
#define CMD_DECLARE(set, t) SET_DECLARE(CMD_SETNAME(set), t)

struct set_concat {
	void **begin;
	void **limit;
};
void set_concat_add(struct set_concat *m, void *begin, void *end);

#define SET_CONCAT_DEF(set, t) 							\
static struct set_concat set ## _concat;					\
static inline const t * const *set ## _begin(void) { return ((const t * const *)set ## _concat.begin); }	\
static inline const t * const *set ## _limit(void) { return ((const t * const *)set ## _concat.limit); }	\
void add_to_ ## set(t **b, t **e)						\
{										\
	set_concat_add(&set ## _concat, b, e);					\
}
#define SET_CONCAT_DECL(set, t)							\
	void add_to_ ## set(t **b, t **e)
SET_CONCAT_DECL(top, struct cmd_function);

int arg_parse(int argc, char * const * argv, const char *desc, const struct opts *);
void arg_help(int argc __unused, char * const *argv, const char *desc, const struct opts *opts);
void usage(const struct cmd_function *f);
void gen_usage_set(const struct cmd_function * const *f, const struct cmd_function * const *flimit);
void cmd_init(void);
void cmd_load_dir(const char *dir, cmd_load_cb_t cb, void *argp);
void dispatch_set(int argc, char *argv[], const struct cmd_function * const *tbl,
    const struct cmd_function * const *tbl_limit);
#define DISPATCH(argc, argv, set)					\
	dispatch_set(argc, argv,					\
	    (const struct cmd_function * const *)CMD_BEGIN(set),	\
	    (const struct cmd_function * const *)CMD_LIMIT(set))	\

#endif /* ARG_H */
