zephyr_library()
zephyr_library_sources(lc709204f.c lc709204f_common.c)
zephyr_library_sources_ifdef(CONFIG_LC709204F_TRIGGER   lc709204f_trigger.c)