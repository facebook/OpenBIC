/*
 * File:		stdlib.h
 * Purpose:		Function prototypes for standard library functions
 *
 * Notes:
 */

#ifndef _STDLIB_H
#define _STDLIB_H
#include <stddef.h>
/********************************************************************
 * Standard library functions
 ********************************************************************/

int
isspace (int);

int
isalnum (int);

int
isdigit (int);

int
isupper (int);

int
strcasecmp (const char *, const char *);

int
strncasecmp (const char *s1, const char *s2, size_t n);

unsigned long
strtoul (const char *, char **, int);

size_t	 strlen (const char *);

char *
strcat (char *, const char *);

char 	*strncat (char *__restrict, const char *__restrict, size_t);

char *
strcpy (char *, const char *);

char 	*strncpy (char *__restrict, const char *__restrict, size_t);

int
strcmp (const char *, const char *);

int	 strncmp (const char *, const char *, size_t);

void *
memcpy (void *, const void *, unsigned);

void *
memset (void *, int, unsigned);

void
free (void *);
 
void *
malloc (unsigned);

#define RAND_MAX 32767

int
rand (void);

void
srand (int);

int	
atoi (const char *__nptr);

/********************************************************************/

#endif