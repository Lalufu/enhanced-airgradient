from time import time

Import("env")

env.Append(SRC_BUILD_FLAGS=[f"-D__BUILD_TIMESTAMP={int(time())}"])
