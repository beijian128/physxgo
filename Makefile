.PHONY: build clean run test

# Load local config (gitignored)
-include config.env

# Defaults
PHYSX_ROOT   ?= /dev/null
BUILD_TYPE   ?= release

# Library suffix: shared libs use SUFFIX_x64, static libs use just SUFFIX
# release: ""    → libPhysX3_x64.so / libPhysX3Extensions.a
# debug:   DEBUG → libPhysX3DEBUG_x64.so / libPhysX3ExtensionsDEBUG.a
ifeq ($(BUILD_TYPE),release)
  LIB_SUFFIX :=
else
  LIB_SUFFIX := $(BUILD_TYPE)
  # Uppercase first letter: debug→DEBUG, checked→CHECKED, profile→PROFILE
  LIB_SUFFIX := $(shell echo $(LIB_SUFFIX) | tr a-z A-Z)
endif

# Derived paths
PHYSX_INCLUDE   := $(PHYSX_ROOT)/PhysX_3.4/Include
PXSHARED_INCLUDE := $(PHYSX_ROOT)/PxShared/include
PHYSX_BIN       := $(PHYSX_ROOT)/PhysX_3.4/Bin/linux64
PHYSX_LIB       := $(PHYSX_ROOT)/PhysX_3.4/Lib/linux64
PXSHARED_BIN    := $(PHYSX_ROOT)/PxShared/bin/linux64
PXSHARED_LIB    := $(PHYSX_ROOT)/PxShared/lib/linux64

# CGO environment (cgo reads these env vars and appends to #cgo directives)
ifeq ($(BUILD_TYPE),debug)
  export CGO_CXXFLAGS := -D_DEBUG -DPX_DEBUG=1 -DPX_CHECKED=1 -I$(PHYSX_INCLUDE) -I$(PXSHARED_INCLUDE)
else
  export CGO_CXXFLAGS := -DNDEBUG -I$(PHYSX_INCLUDE) -I$(PXSHARED_INCLUDE)
endif
export CGO_LDFLAGS  := \
  -L$(PHYSX_BIN) -L$(PXSHARED_BIN) -L$(PHYSX_LIB) -L$(PXSHARED_LIB) \
  -lPhysX3Extensions$(LIB_SUFFIX) \
  -lPhysX3$(LIB_SUFFIX)_x64 \
  -lPhysX3Common$(LIB_SUFFIX)_x64 \
  -lPhysX3CharacterKinematic$(LIB_SUFFIX)_x64 \
  -lPhysX3Cooking$(LIB_SUFFIX)_x64 \
  -lPxFoundation$(LIB_SUFFIX)_x64 \
  -lPxPvdSDK$(LIB_SUFFIX)_x64 \
  -Wl,-rpath,$(PHYSX_BIN) -Wl,-rpath,$(PXSHARED_BIN)

OUTPUT := bin/physx-demo
WEB_OUTPUT := bin/physx-webdemo

build:
	@echo "=== Building with BUILD_TYPE=$(BUILD_TYPE) ($(LIB_SUFFIX)) ==="
	go build -o $(OUTPUT) ./cmd/

web:
	@echo "=== Building web demo with BUILD_TYPE=$(BUILD_TYPE) ==="
	go build -o $(WEB_OUTPUT) ./cmd/webdemo/

clean:
	rm -f $(OUTPUT) $(WEB_OUTPUT)

run: build
	$(OUTPUT) $(ARGS)

test: build
	$(OUTPUT) raycast
