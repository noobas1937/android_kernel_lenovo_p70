#ifndef __MTK_SELINUX_WARNING_LIST_H__
#define __MTK_SELINUX_WARNING_LIST_H__

#ifdef CONFIG_SECURITY_SELINUX_DEVELOP
extern int selinux_enforcing;
#else
#define selinux_enforcing 1
#endif

#define AEE_FILTER_NUM 70
const char *aee_filter_list[AEE_FILTER_NUM] = {
/*	"u:r:adbd:s0", */
	"u:r:zygote:s0",
	"u:r:netd:s0",
	"u:r:installd:s0",
	"u:r:vold:s0",
};

#endif
