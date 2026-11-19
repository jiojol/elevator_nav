# Share the mapping output defaults: each successful map replaces dog.*,
# and the next navigation reads that same complete map set.
# Explicit DOG_NAV_* overrides can still select another map deliberately.
_NAV_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${_NAV_ENV_DIR}/env.sh"
