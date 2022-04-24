# Be silent per default, but 'make V=1' will show all compiler calls.
V?=0
ifeq ($(V),0)
	Q	:= @
	NULL	:= 2>/dev/null
endif

ifeq ($(TARGET),SEND)
TARGET_EXEC ?= send.out
else
TARGET_EXEC ?= receive.out
endif

BUILD_DIR ?= build
SRC_DIRS ?= src
INC_DIRS ?= include
SRC_TYPES := cpp c s
SRCS := $(foreach dir, $(SRC_DIRS), $(foreach type, $(SRC_TYPES), $(wildcard $(dir)/*.$(type))))
OBJS := $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(SRCS))))
DEPS := $(OBJS:.o=.d)
INC_DIRS += $(dir $(SRC_DIRS))
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
CPPFLAGS ?= $(INC_FLAGS) -MMD -MP -ggdb3

ifeq ($(TARGET),SEND)
CPPFLAGS+= -DTEST_XMODEM_SEND
else
CPPFLAGS+= -DTEST_XMODEM_RECEIVE
endif

$(TARGET_EXEC): Check_target $(OBJS)
	@printf " LD\t$@\n"
	$(Q)$(CC) $(OBJS) -o $@ $(LDFLAGS)

# assembly
(BUILD_DIR)/%.o: %.s
	$(Q)$(MKDIR_P) $(dir $@)
	@printf " AS\t$<\n"
	$(Q)$(AS) $(ASFLAGS) -c $< -o $@

# c source
$(BUILD_DIR)/%.o: %.c
	$(Q)$(MKDIR_P) $(dir $@)
	@printf " CC\t$<\n"
	$(Q)$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# c++ source
$(BUILD_DIR)/%.o: %.cpp
	$(Q)$(MKDIR_P) $(dir $@)
	@printf " CXX\t$<\n"
	$(Q)$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

Check_target:
	$(MAKE) clean

.PHONY: clean
clean:
	$(RM) -r $(BUILD_DIR)
-include $(DEPS)
MKDIR_P ?= mkdir -p
