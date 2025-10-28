cmd_/home/ucas/1406/1406_OBC/drv/int_drv/modules.order := {   echo /home/ucas/1406/1406_OBC/drv/int_drv/int_drv.ko; :; } | awk '!x[$$0]++' - > /home/ucas/1406/1406_OBC/drv/int_drv/modules.order
