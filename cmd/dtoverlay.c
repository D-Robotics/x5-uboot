#include <common.h>
#include <command.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <asm/global_data.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <mapmem.h>
#include <asm/io.h>
#include <linux/string.h>
#include <fdt_support.h>
#include <malloc.h>
#include <dtoverlay.h>
#include <fs.h>
#include <mmc.h>

#include <hb_utils.h>

uint32_t hb_dev_name_get(char *devname)
{
	char *fdtfile;
	size_t len;

	fdtfile = env_get("fdtfile");
	if (!fdtfile) {
		return 1;
	}

	len = strlen(fdtfile);

	strncpy(devname, fdtfile, len - 4);
	return 0;
}

static int do_dtoverlay(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
    char *data, *dp, *lp, *ptr, *cfg_file;
    ulong file_addr, dt_addr, overlay_addr;
    loff_t size = 0;
    int param_num = 0;
    DTPARAM_T dtparam[16];
    char line[128], filter_name[32];
    int length;
    char *comma, *token, *equal_sign;
    int som_type = 0;
    char devname[32];

    if (argc < 5)
        return CMD_RET_USAGE;

    dtoverlay_debug("param no:%d  %s  %s %s %s \n", argc, argv[1], argv[2], argv[3], argv[4]);

    dt_addr = simple_strtoul(argv[1], NULL, 16);

    overlay_addr = simple_strtoul(argv[2], NULL, 16);

    cfg_file = argv[3];

    file_addr = simple_strtoul(argv[4], NULL, 16);

    ptr = map_sysmem(file_addr, 0);
 
    if(hb_ext4_load(cfg_file, file_addr, &size) != 0)
    {
        dtoverlay_error("can't load config file(%s)\n", cfg_file);
        return 0;
    }
    if ((data = malloc(size + 1)) == NULL)
    {
        dtoverlay_debug("can't malloc %lu bytes\n", (ulong)size + 1);
        __set_errno(ENOMEM);
        return 0;
    }

    memcpy(data, ptr, size);
    data[size] = '\0';
    dp = data;

    memset(devname, 0, sizeof(devname));

    fdt_shrink_to_minimum((void *)dt_addr, 0x1000);

    while ((length = hb_getline(line, sizeof(line), &dp)) > 0) {
        dtoverlay_debug("Line length: %d, Read line: %s\n", length, line);

        if (length == -1) {
            break;
        }

        lp = line;

        /* skip leading white space */
        while (isblank(*lp))
            ++lp;

        /* skip comment lines */
        if (*lp == '#' || *lp == '\0')
            continue;

        /* Skip items that do not match the filter */
        if (som_type != 0 && strncmp(lp, "[", 1) != 0 )
            continue;

        if (strncmp(lp, "dtparam=", 8) == 0)
        {
            lp += strlen("dtparam=");
            token = strtok(lp, ",");

            while (token != NULL) {
                equal_sign = strchr(token, '=');
                
                if (equal_sign != NULL) {
                    *equal_sign = '\0';
                    strncpy(dtparam[param_num].name, strim(token), sizeof(dtparam[param_num].name));
                    strncpy(dtparam[param_num].value, strim(equal_sign + 1), sizeof(dtparam[param_num].value));
                }
                else
                {
                    strncpy(dtparam[param_num].name, strim(token), sizeof(dtparam[param_num].name));
                    strncpy(dtparam[param_num].value, "true", sizeof(dtparam[param_num].value));
                }

                dtoverlay_debug("dtparam_name: %s, dtparam_value: %s\n",
                    dtparam[param_num].name, dtparam[param_num].value);
                param_num++;
                token = strtok(NULL, ",");
            }
        }
        else if (strncmp(lp, "dtoverlay=", 10) == 0)
        {
            char overlay_name[64] = {};
            DTPARAM_T overlay_para[16];
            int overlay_para_num = 0;

            lp += strlen("dtoverlay=");

            comma = strchr(lp, ',');
        
            if (comma != NULL) {
                *comma = '\0';
            }

            strncpy(overlay_name, strim(lp), sizeof(overlay_name));
            overlay_name[sizeof(overlay_name) - 1] = '\0';
            dtoverlay_debug("Overlay File Name: %s\n", overlay_name);

            if (comma != NULL) {
                token = strtok(comma + 1, ",");
                
                while (token != NULL) {
                    equal_sign = strchr(token, '=');
                    
                    if (equal_sign != NULL) {
                        *equal_sign = '\0';
                        strncpy(overlay_para[overlay_para_num].name,
                            strim(token), sizeof(overlay_para[overlay_para_num].name));
                        strncpy(overlay_para[overlay_para_num].value,
                            strim(equal_sign + 1), sizeof(overlay_para[overlay_para_num].value));
                    }
                    else
                    {
                        strncpy(overlay_para[overlay_para_num].name,
                            strim(token), sizeof(overlay_para[overlay_para_num].name));
                        strncpy(overlay_para[overlay_para_num].value,
                            "true", sizeof(overlay_para[overlay_para_num].value));
                    }
                    dtoverlay_debug("overlay_para_name: %s, overlay_para_value: %s\n",
                        overlay_para[overlay_para_num].name, overlay_para[overlay_para_num].value);
                    overlay_para_num++;

                    token = strtok(NULL, ",");
                }
            }
            if(strlen(overlay_name) != 0)
            {
                do_dtparam(overlay_addr, overlay_name, overlay_para, overlay_para_num);
                fdt_overlay_apply_verbose((void *)dt_addr, (void *)overlay_addr);
            }
        }
        else if (strncmp(lp, "dtdebug=", 8) == 0)
        {
            if (strncmp(lp, "dtdebug=0", 9) == 0)
                dtoverlay_enable_debug(0);
            else
                dtoverlay_enable_debug(1);
        }
        else if (strncmp(lp, "ion=", 4) == 0)
        {
            lp += strlen("ion=");
            token = strtok(lp, ",");

            while (token != NULL) {
                equal_sign = strchr(token, '=');
                
                if (equal_sign != NULL) {
                    *equal_sign = '\0';
                    dtoverlay_debug("ion %s, %s\n",token,equal_sign + 1);
                    env_set(token, equal_sign + 1);
                }
                token = strtok(NULL, ",");
            }
        }
        else if (strncmp(lp, "[", 1) == 0)
        {

            hb_extract_filter_name(lp, filter_name);
            hb_dev_name_get(devname);
            if (strcmp("all", filter_name) == 0 || strcmp("devname", filter_name) == 0)
                som_type = 0;
            else
                som_type = 1;
            printk("######################## %s %s %d filter_name %s devname %s som_type %d\n",__FILE__, __FUNCTION__, __LINE__,filter_name,devname,som_type);

        }
    }

    if (param_num != 0)
    {
        do_dtparam(dt_addr, "", dtparam, param_num);
    }
    free(data);
    return 0;
}

static char dtoverlay_help_text[] =
    "dtoverlay <dt addr> <overlays addr> <config file> <config addr>  - load the config.txt to <fileaddr>\n";

U_BOOT_CMD(
    dtoverlay, 255, 0, do_dtoverlay,
    "dtoverlay utility commands", dtoverlay_help_text);