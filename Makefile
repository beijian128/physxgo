.PHONY: build clean run

# Load local config (gitignored file with absolute paths)
-include config.env

# Ensure PHYSX_ROOT is set
ifndef PHYSX_ROOT
$(error PHYSX_ROOT is not set. Copy config.env.example to config.env and edit it.)
endif

# Derive paths from PHYSX_ROOT
PHYSX_INCLUDE  := $(PHYSX_ROOT)/PhysX_3.4/Include
PXSHARED_INCLUDE := $(PHYSX_ROOT)/PxShared/include
PHYSX_BIN      := $(PHYSX_ROOT)/PhysX_3.4/Bin/linux64
PHYSX_LIB      := $(PHYSX_ROOT)/PhysX_3.4/Lib/linux64
PXSHARED_BIN   := $(PHYSX_ROOT)/PxShared/bin/linux64
PXSHARED_LIB   := $(PHYSX_ROOT)/PxShared/lib/linux64

# Export for cgo
export CGO_CXXFLAGS := -I$(PHYSX_INCLUDE) -I$(PXSHARED_INCLUDE)
export CGO_LDFLAGS  := -L$(PHYSX_BIN) -L$(PXSHARED_BIN) -L$(PHYSX_LIB) -L$(PXSHARED_LIB)
export CGO_LDFLAGS  += -Wl,-rpath,$(PHYSX_BIN) -Wl,-rpath,$(PXSHARED_BIN)

OUTPUT := bin/physx-demo

build:
	go build -o $(OUTPUT) ./cmd/

clean:
	rm -f $(OUTPUT)

run: build
	$(OUTPUT) $(ARGS)

# Quick test: raycast demo (no PVD needed)
test: build
	$(OUTPUT) raycast
