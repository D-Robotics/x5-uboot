/*
 * COPYRIGHT NOTICE
 * Copyright 2023 Horizon Robotics, Inc.
 * All rights reserved.
 */

#ifndef __HB_UTILS_H__
#define __HB_UTILS_H__

extern char hb_filter_names[][20];
int hb_get_som_type_by_filter_name(char *filter_name);
int hb_extract_filter_name(const char* line, char* filter_name);

int hb_ext4_load(char *filename, unsigned long addr, loff_t *len_read);
int hb_getline(char str[], int lim, char **mem_ptr);
int atoi(const char * str);

#endif /* __HB_UTILS_H__ */
