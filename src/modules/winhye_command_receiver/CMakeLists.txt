px4_add_module(
	MODULE modules__winhye_command_receiver
	MAIN winhye_command_receiver
	STACK_MAIN 2000
	SRCS
		winhye_command_receiver.c
	DEPENDS
	)

