zephyr_library()

zephyr_library_sources_ifdef(CONFIG_RTC_RV3028 rtc_rv3028.c)