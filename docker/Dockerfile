# syntax=docker/dockerfile:1
#
# droneengage_simulation_land - all-in-one scenario playground image.
#
#   docker build -f docker/Dockerfile -t de-sim .
#   docker run -it --network host de-sim view docker/canopy_quickstart.yaml
#
# Everything the sim launches is baked in: ArduPilot SITL, the DE
# binaries (de_comm / de_mavlink) and the Mavlink3DMap viewer (the
# databus python client is the vendored tools/rpi/de_databus_py copy).
# All upstream refs are pinned below so the image is reproducible - no
# host checkouts are mounted or copied.
#
# The public de_comm/de_mavlink mirrors predate the mission supervisor,
# so scenarios that `do: deploy` (ops/packages) cannot run here; the
# image's own demo (docker/canopy_quickstart.yaml) flies the canopy
# sweep through the classic mt-1075 mission upload instead. Two image
# patches below adapt the run to the public builds: the catalog ready
# marker and the missing databus message ids.
#
# Op verification (de.operation docs) is not in this image: it needs
# the webclient's verify pipeline, which the public
# droneengage_webclient_react mirror does not publish. verify_wrap.js
# is dockerignored out of the image, so the runner takes its designed
# "P5-06 CLI not yet available" skip path and still writes verify.json.
#
# Pinned upstream refs (tags or full commit shas - rebuild with
#   --build-arg X_REF=<sha> to move one).
ARG ARDUPILOT_REF=Copter-4.6.3
ARG DE_COMM_REF=206b5a2f2a740e032e301040ff92a7a5ff15d7a5
ARG DE_MAVLINK_REF=be5d8a5701c19c6c4b4164636522591cdf7733b8
ARG M3D_REF=309b2f9090ec5b7c51a7448b356031ee84c85c8a
# SITL vehicles to build - the canopy demo needs copter only; space
# separates ("copter plane rover") for a fuller image.
ARG SITL_VEHICLES="copter"

# ------------------------------------------------------------ SITL build
FROM ubuntu:24.04 AS sitl-build
ARG ARDUPILOT_REF
ARG SITL_VEHICLES
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git ca-certificates python3 python3-pip \
        python3-setuptools \
    && rm -rf /var/lib/apt/lists/*
# SITL build-time python deps (mavgen + waf helpers)
RUN pip3 install --break-system-packages --no-cache-dir \
        pymavlink pexpect pyserial empy==3.3.4 lxml future
RUN git clone -q --depth 1 --branch "${ARDUPILOT_REF}" \
        https://github.com/ArduPilot/ardupilot /opt/ardupilot
WORKDIR /opt/ardupilot
# SITL needs a subset of the submodules - everything except ChibiOS and
# CrashDebug (MCU-only).
RUN git submodule update --init --depth 1 --recommend-shallow \
        modules/waf modules/mavlink modules/gtest modules/gbenchmark \
        modules/gsoap modules/lwip \
        modules/Micro-XRCE-DDS-Client modules/Micro-CDR \
        modules/DroneCAN/DSDL modules/DroneCAN/pydronecan \
        modules/DroneCAN/dronecan_dsdlc modules/DroneCAN/libcanard
RUN ./waf configure --board sitl && ./waf ${SITL_VEHICLES}

# ------------------------------------------------- DE modules (C++ build)
FROM ubuntu:24.04 AS de-build
ARG DE_COMM_REF
ARG DE_MAVLINK_REF
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates \
        libboost-system-dev libboost-thread-dev libboost-chrono-dev \
        libboost-date-time-dev libboost-coroutine-dev \
        libboost-context-dev \
        libcurl4-openssl-dev libssl-dev \
    && rm -rf /var/lib/apt/lists/*
COPY docker/fetch-repo.sh /usr/local/bin/fetch-repo
RUN chmod +x /usr/local/bin/fetch-repo
# de_comm
RUN fetch-repo https://github.com/DroneEngage/droneengage_communication \
        "${DE_COMM_REF}" /opt/de_code/droneengage_communication
RUN cd /opt/de_code/droneengage_communication && ./build.sh RELEASE
# de_mavlink (binary lands as bin/de_ardupilot); the droneengage_common
# submodule is public.
RUN fetch-repo https://github.com/DroneEngage/droneengage_mavlink \
        "${DE_MAVLINK_REF}" /opt/de_code/droneengage_mavlink \
        src/de_common \
 && cd /opt/de_code/droneengage_mavlink && ./build.sh RELEASE

# --------------------------------------------------- Mavlink3DMap viewer
# vite dev server + udp2websocket bridge; npm workspaces hoist vite to
# the repo root node_modules, which is where engine/simlib.py looks.
FROM node:22-bookworm-slim AS m3d
ARG M3D_REF
RUN apt-get update && apt-get install -y --no-install-recommends git \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY docker/fetch-repo.sh /usr/local/bin/fetch-repo
RUN chmod +x /usr/local/bin/fetch-repo \
 && fetch-repo https://github.com/HefnySco/Mavlink3DMap \
        "${M3D_REF}" /opt/de_code/Mavlink3DMap2
RUN cd /opt/de_code/Mavlink3DMap2 \
 && npm install --no-audit --no-fund

# ----------------------------------------------------------------- image
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-venv openssl ca-certificates git bash \
        procps \
        libstdc++6 \
        libboost-system1.83.0 libboost-thread1.83.0 \
        libboost-chrono1.83.0 libboost-date-time1.83.0 \
        libboost-coroutine1.83.0 libboost-context1.83.0 \
        libcurl4 libssl3 \
    && rm -rf /var/lib/apt/lists/*

# node (vite dev server, udp2websocket bridge)
COPY --from=node:22-bookworm-slim /usr/local/bin/node /usr/local/bin/node
COPY --from=node:22-bookworm-slim /usr/local/lib/node_modules \
        /usr/local/lib/node_modules
RUN ln -s /usr/local/lib/node_modules/npm/bin/npm-cli.js \
        /usr/local/bin/npm \
 && ln -s /usr/local/lib/node_modules/npm/bin/npx-cli.js \
        /usr/local/bin/npx \
 && ln -s /usr/local/lib/node_modules/corepack/dist/corepack.js \
        /usr/local/bin/corepack || true

# python deps for engine/ + stubs/ + the databus python client
RUN python3 -m venv /opt/venv \
 && /opt/venv/bin/pip install --no-cache-dir \
        websockets pymavlink pyyaml colorama
ENV PATH="/opt/venv/bin:/usr/local/bin:${PATH}"

# the sim engine itself (build context = this repo)
WORKDIR /opt/de_code/droneengage_simulation_land
COPY . .

# fetched / built dependencies laid out the way sim.local.env expects
COPY --from=de-build /opt/de_code/droneengage_communication/bin \
        /opt/de_code/droneengage_communication/bin
COPY --from=de-build /opt/de_code/droneengage_mavlink/bin \
        /opt/de_code/droneengage_mavlink/bin
COPY --from=de-build \
        /opt/de_code/droneengage_mavlink/de_mavlink.config.module.json \
        /opt/de_code/droneengage_mavlink/de_mavlink.config.module.json
COPY --from=m3d /opt/de_code/Mavlink3DMap2 \
        /opt/de_code/Mavlink3DMap2
# SITL: the binaries plus the --defaults parm trees the engine names
# relative to ARDUPILOT_DIR
COPY --from=sitl-build /opt/ardupilot/build/sitl/bin \
        /opt/ardupilot/build/sitl/bin
COPY --from=sitl-build /opt/ardupilot/Tools/autotest/default_params \
        /opt/ardupilot/Tools/autotest/default_params
COPY --from=sitl-build /opt/ardupilot/Tools/autotest/models \
        /opt/ardupilot/Tools/autotest/models

# image-local machine file: every path resolves inside the container;
# environment variables still win over these (simlib setting()).
RUN printf '%s\n' \
        'DE_CODE=/opt/de_code' \
        'ARDUPILOT_DIR=/opt/ardupilot' \
        'MAVLINK3DMAP_DIR=/opt/de_code/Mavlink3DMap2' \
        'SIM_M3D_VERSION=1.4.1' \
        > sim.local.env

# stubs import the databus python client from $DE_DATABUS_DIR/python.
# The public droneengage_databus mirror predates the message ids the
# stubs use (6544/6545/6546), so the image points DE_DATABUS_DIR at the
# vendored, identical client this repo already ships for the RPi tools.
RUN mkdir -p /opt/de_code/de_databus \
 && ln -s ../droneengage_simulation_land/tools/rpi/de_databus_py \
        /opt/de_code/de_databus/python

# The public de_mavlink has no nav adapter, so 'nav adapter hello' is
# never journaled and every ready_re wait would time out. Swap the
# catalog's journal marker for the stdout line every de_common module
# prints when it finds its broker. Image copy only - the repo file is
# untouched (the private build still emits the line too, so the patched
# marker stays correct when the mirrors catch up).
RUN sed -i 's/^  ready_re: "nav adapter hello"$/  ready_log_re: "Communicator Server Found"/' \
        modules/catalog.yaml \
 && grep -q 'ready_log_re: "Communicator Server Found"' \
        modules/catalog.yaml

COPY docker/entrypoint.sh /opt/de_code/droneengage_simulation_land/docker/entrypoint.sh
RUN chmod +x docker/entrypoint.sh sim stubs/*.py
ENTRYPOINT ["/opt/de_code/droneengage_simulation_land/docker/entrypoint.sh"]
CMD ["view", "docker/canopy_quickstart.yaml"]
