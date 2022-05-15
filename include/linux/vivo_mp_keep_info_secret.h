/*
 * =====================================================================================
 *
 *       Filename:  vivo_mp_keep_info_secret.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  08/04/17 18:26:23
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  chen yuqin
 *        Company:  vivo
 *
 * =====================================================================================
 */



#ifndef _VIVO_MP_KEEP_INFO_SECRET_H
#define _VIVO_MP_KEEP_INFO_SECRET_H

#ifdef CONFIG_VIVO_MP_KEEP_INFO_SECRET

#include <linux/sched.h>
#include <linux/cred.h>
#include <linux/mm.h>
#include <linux/string.h>

#if 1

static char app_while_list[128][48] = {                 // quite gross for 'static'
    "com.android.cts.priv.ctsshim", 
    // "com.vivo.weather.provider", 
    // "com.qualcomm.qti.auth.sampleextauthservice",
    "com.android.providers.calendar", 
    // "com.bbk.scene.launcher.theme", 
    "android.process.media",                            // "com.android.providers.media", 
    // "com.vivo.livewallpaper.coffeetime", 
    // "com.vivo.easyshare", 
    // "com.qualcomm.shutdownlistner", 
    "com.android.wallpapercropper", 
    // "com.vivo.safecenter", 
    // "com.vivo.collage", 
    // "com.vivo.compass", 
    // "com.vivo.mediatune", 
    "com.android.documentsui", 
    "com.android.externalstorage", 
    "com.android.htmlviewer", 
    // "com.qualcomm.ftm", 
    // "com.vivo.numbermark", 
    "com.android.providers.downloads", 
    // "com.qualcomm.qti.auth.sampleauthenticatorservice", 
    "com.android.bbkmusic", 
    "com.qapp.secprotect", 
    // "com.bbk.VoiceOneshot", 
    // "com.qualcomm.qti.telephonyservice", 
    // "com.vivo.msgpush", 
    "com.android.browser", 
    // "com.bbk.updater", 
    "com.android.bbkcalculator", 
    // "com.vivo.ewarranty", 
    "com.android.defcontainer", 
    "com.android.pacprocessor", 
    "com.android.filemanager", 
    "com.android.certinstaller", 
    // "com.vivo.secime.service", 
    // "com.vivo.upnpserver", 
    // "com.bbk.cloud", 
    // "com.bbk.VoiceAssistant", 
    // "com.bbk.theme", 
    "com.android.egg", 
    "com.android.mms", 
    "com.android.mtp", 
    "com.android.backupconfirm", 
    // "com.bbk.calendar", 
    "com.android.statementservice", 
    // "com.bbk.iqoo.feedback", 
    "com.android.BBKClock", 
    "com.android.photoeditor", 
    // "com.vivo.quickpay", 
    // "com.vlife.vivo.wallpaper", 
    "com.ringclip", 
    "com.android.BBKPhoneInstructions", 
    "com.android.sharedstoragebackup", 
    "com.android.printspooler", 
    // "com.vivo.dream.note", 
    // "com.vivo.Tips", 
    // "com.vivo.game", 
    // "com.qualcomm.qti.qcom_accesslogkit", 
    "com.android.frameworks.telresources", 
    "com.baidu.input_bbk.service", 
    // "com.bbk.photoframewidget", 
    "com.android.skin", 
    "com.ibimuyu.lockscreen", 
    // "com.bbk.iqoo.logsystem", 
    // "com.vivo.SmartKey", 
    "com.google.android.webview", 
    "android.ext.shared", 
    // "com.vivo.videoeditor", 
    "com.google.android.syncadapters.contacts", 
    "com.android.camera", 
    "com.android.printservice.recommendation", 
    // "com.bbk.launcher2", 
    "com.android.dialer", 
    "com.google.android.gsf", 
    "android.ext.services", 
    "com.android.calllogbackup", 
    // "com.vivo.faceunlock", 
    "com.android.packageinstaller", 
    "com.svox.pico", 
    // "com.vivo.livewallpaper.silk", 
    "com.android.proxyhandler", 
    // "com.vivo.widget.calendar", 
    // "com.vivo.magazine", 
    "com.android.VideoPlayer", 
    "com.android.musiceffecttest", 
    // "com.vivo.devicereg", 
    "com.google.android.syncadapters.calendar", 
    // "com.vivo.doubletimezoneclock", 
    "com.android.managedprovisioning", 
    // "com.iqoo.secure", 
    // "com.vivo.childrenmode", 
    // "com.vivo.sim.contacts", 
    "com.android.skintwo", 
    // "com.vivo.sdkplugin", 
    // "com.bbk.appstore", 
    "com.android.smspush", 
    // "com.vivo.findphone", 
    // "com.android.bbk.lockscreen3", 
    // "com.vivo.audiofx", 
    // "com.vivo.dream.weather", 
    // "com.bbk.theme.resources", 
    "com.android.storagemanager", 
    "com.android.cts.ctsshim", 
    // "com.vivo.dream.clock", 
    // "com.vivo.dream.music", 
    "com.android.vpndialogs", 
    // "com.vivo.gallery", 
    // "com.vivo.email", 
    "com.qti.vzw.ims.internal.tests", 
    // "com.vivo.space", 
    "com.android.notes", 
    "com.android.providers.blockednumber", 
    "com.android.providers.userdictionary", 
    "com.android.systemui", 
    // "com.bbk.scene.tech", 
    // "com.vivo.livewallpaper.coralsea", 
    "com.android.bluetoothmidiservice", 
    // "com.vivo.browser", 
    "com.focaltech.ft_terminal_test", 
    // "com.vivo.motionrecognition", 
    "com.android.skinfive", 
    "com.android.skinfour", 
    // "com.qualcomm.embms", 
    "com.android.providers.contacts", 
    "com.android.captiveportallogin", 
    "com.android.skinthree", 
    "com.chaozh.iReader", 
    "com.android.bbksoundrecorder", 
    // "com.bbk.account", 

    "android.process.acore", 
    "/system/bin/dex2oat", 
    "app_process", 
    // "/system/bin/cat", 
    "eu.chainfire.perfmon", 
    "com.autonavi.minimap", 
    "com.jm.android.jumei", 
    "com.qiyi.video", 
    "com.sankuai.meituan", 
    "com.sina.weibo", 
    "com.taobao.taobao", 
    "com.tencent.mm", 
    "com.tencent.mobileqq", 
    "com.tencent.news", 
    "com.tencent.qqlive", 
    "com.wuba", 
    "ctrip.android.view", 
    "com.wyd.hero.yqlfc.cb1.vivo", 
    "com.gamebench.metricscollector", 
    "com.mobiletools.systemhelper", 
};

#endif

static inline int should_keep_confidentiality(void) {
#if 1
    const struct cred *cred;
    char app_name[48] = {0};
    int i;
    int should = 0;
    cred = get_task_cred(current);
    if (cred->uid.val >= 10000) {                       // loki: uid or euid?
        get_cmdline(current, app_name, sizeof(app_name));
        app_name[sizeof(app_name)-1] = 0;
        if (strstr(app_name, ".vivo.") || 
            strstr(app_name, ".bbk.") || 
            strstr(app_name, ".iqoo.") || 
            strstr(app_name, ".qualcomm.")) {
            ;
        } else {
#if 1
            // make sure no ':' in app_name
            for (i = 0; i < sizeof(app_name) && app_name[i]; ++i) {
                if (app_name[i] == ':') {
                    app_name[i] = 0;
                    break;
                }
            }
#endif
            for (i = 0; i < (int) (sizeof(app_while_list)/sizeof(app_while_list[0])) && app_while_list[i][0]; ++i) {
                if (strcmp(app_name, app_while_list[i]) == 0) {
                    break;
                }
            }
            if (i >= (int) (sizeof(app_while_list)/sizeof(app_while_list[0])) || 
                app_while_list[i][0] == 0) {
                // printk(KERN_ERR "should keep confidentiality for [%s]\n", app_name);
                should = 1;
            }
        }
    }
    put_cred(cred);
    return should;
#else
    return 1;
#endif
}

#endif

#endif /* _VIVO_MP_KEEP_INFO_SECRET_H */
