#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

// 获取HTML网页内容
const char* get_web_page_html(void);
const char* get_debug_page_html(void);

// 获取电机状态JSON数据
const char* get_motor_status_json(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_INTERFACE_H