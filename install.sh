#!/usr/bin/env bash
# Install the planar controller on a Linux machine and start it with Docker Compose.
#
# On a fresh machine:
#   curl -fsSL https://raw.githubusercontent.com/AAUSmartProductionLab/planar-controller/main/install.sh \
#     | bash -s -- --pmc-ip 192.168.10.100
# From a clone of the repository:
#   ./install.sh --pmc-ip 192.168.10.100
#
# Run it again to update: it pulls, rebuilds and restarts, and keeps .env (flags change it).
#
# Options:
#   --pmc-ip IP       the PMC's address (empty: search the local network)
#   --port N          the OPC UA port (4840)
#   --shuttles LIST   the xbot ids to offer, e.g. 1,2,3 (empty: the ones the PMC knows)
#   --loading-station X,Y  in mm: free xbots queue up there, and a Reset sends them there
#   --idle-rz DEG     a multiple of 90: free xbots are turned to it, and a Reset turns them all
#   --mode free|line  anywhere on the table, or only on the lanes of --line-layout (free)
#   --line-layout FILE|URL  line mode: the layout, a file in config/ (config/NAME.json) or a URL
#   --autostart       Reset and Start the table when the controller starts
#   --no-autostart    leave it Stopped until it is Reset and Started over OPC UA (the default)
#   --dir DIR         where to put the repository when not run from one (~/planar-controller)
#   --branch NAME     the branch to install (main)
#   --no-start        install and build, but do not start
#   -h, --help        this text
set -euo pipefail

REPO_URL="https://github.com/AAUSmartProductionLab/planar-controller.git"
DIR="${HOME}/planar-controller"
BRANCH="main"
START=1
declare -A SETTINGS=()

usage() { sed -n '2,/^set -euo/p' "$0" | sed '$d; s/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
    case "$1" in
        --pmc-ip) SETTINGS[PMC_IP]="$2"; shift 2 ;;
        --port) SETTINGS[OPCUA_PORT]="$2"; shift 2 ;;
        --shuttles) SETTINGS[SHUTTLES]="$2"; shift 2 ;;
        --loading-station) SETTINGS[LOADING_STATION]="$2"; shift 2 ;;
        --idle-rz) SETTINGS[IDLE_RZ]="$2"; shift 2 ;;
        --mode) SETTINGS[MODE]="$2"; shift 2 ;;
        --line-layout) SETTINGS[LINE_LAYOUT]="$2"; shift 2 ;;
        --autostart) SETTINGS[AUTOSTART]=1; shift ;;
        --no-autostart) SETTINGS[AUTOSTART]=0; shift ;;
        --dir) DIR="$2"; shift 2 ;;
        --branch) BRANCH="$2"; shift 2 ;;
        --no-start) START=0; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
done

say() { printf '\033[1m==> %s\033[0m\n' "$*"; }
die() { printf 'error: %s\n' "$*" >&2; exit 1; }

[ "$(uname -s)" = "Linux" ] || die "this installer is for Linux; elsewhere use docker compose up -d --build"
if [ "$(id -u)" -eq 0 ]; then SUDO=""; else
    command -v sudo >/dev/null || die "run as root or install sudo"
    SUDO="sudo"
fi

apt_install() {
    command -v apt-get >/dev/null || return 1
    $SUDO apt-get update -qq
    $SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y -qq "$@"
}

# --- git and Docker with the compose plugin -----------------------------------------------------
if ! command -v git >/dev/null; then
    say "Installing git"
    apt_install git || die "install git, then run this again"
fi

if ! command -v docker >/dev/null; then
    say "Installing Docker"
    # Ubuntu's own packages where they exist, else Docker's installer
    if ! apt_install docker.io docker-compose-v2 docker-buildx 2>/dev/null; then
        curl -fsSL https://get.docker.com | $SUDO sh
    fi
fi
if ! docker compose version >/dev/null 2>&1 && ! $SUDO docker compose version >/dev/null 2>&1; then
    say "Installing the docker compose plugin"
    apt_install docker-compose-v2 2>/dev/null || apt_install docker-compose-plugin \
        || die "install the docker compose plugin, then run this again"
fi
if command -v systemctl >/dev/null; then
    $SUDO systemctl enable --now docker >/dev/null 2>&1 || true
fi
if [ -n "$SUDO" ] && ! id -nG "$USER" | grep -qw docker; then
    $SUDO usermod -aG docker "$USER" || true
    echo "Added $USER to the docker group (takes effect at the next login)."
fi
# This shell may not be in the docker group yet: fall back to sudo for docker itself
if docker info >/dev/null 2>&1; then DOCKER="docker"; else DOCKER="$SUDO docker"; fi

# --- the repository -------------------------------------------------------------------------------
SELF="${BASH_SOURCE[0]:-}"
if [ -n "$SELF" ] && [ -f "$SELF" ] && [ -f "$(dirname "$SELF")/docker-compose.yml" ]; then
    DIR="$(cd "$(dirname "$SELF")" && pwd)"
    say "Using the repository at $DIR"
elif [ -d "$DIR/.git" ]; then
    say "Updating $DIR"
    git -C "$DIR" fetch --quiet origin "$BRANCH"
    git -C "$DIR" checkout --quiet "$BRANCH"
    git -C "$DIR" pull --quiet --ff-only origin "$BRANCH"
else
    say "Cloning into $DIR"
    git clone --quiet --branch "$BRANCH" "$REPO_URL" "$DIR"
fi
cd "$DIR"

# --- settings -------------------------------------------------------------------------------------
if [ ! -f .env ]; then
    cp .env.example .env
    say "Created .env from .env.example"
fi
for key in "${!SETTINGS[@]}"; do
    value="${SETTINGS[$key]}"
    if grep -qE "^${key}=" .env; then
        escaped=$(printf '%s' "$value" | sed 's/[&|\\]/\\&/g')  # a URL may hold & or |
        sed -i "s|^${key}=.*|${key}=${escaped}|" .env
    else
        printf '%s=%s\n' "$key" "$value" >> .env
    fi
done
set -a
# shellcheck disable=SC1091
. ./.env
set +a

# --- build and start ------------------------------------------------------------------------------
say "Building the image (the first build takes a few minutes: mono is large)"
$DOCKER compose build
if [ "$START" -eq 0 ]; then
    say "Built. Start it with: cd $DIR && docker compose up -d"
    exit 0
fi
say "Starting"
$DOCKER compose up -d

PORT="${OPCUA_PORT:-4840}"
for _ in $(seq 1 60); do
    if (exec 3<>"/dev/tcp/127.0.0.1/${PORT}") 2>/dev/null; then break; fi
    sleep 1
done
HOST="$(hostname -I 2>/dev/null | awk '{print $1}')"
$DOCKER compose ps
echo
echo "OPC UA:   opc.tcp://${HOST:-localhost}:${PORT}/   (namespace urn:aau:smartproductionlab:planar)"
echo "PMC:      ${PMC_IP:-auto search}"
echo "Settings: $DIR/.env  (then: docker compose up -d)"
echo "Log:      cd $DIR && docker compose logs -f"
