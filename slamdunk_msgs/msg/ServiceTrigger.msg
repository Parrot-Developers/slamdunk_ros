byte TRIGGER_MIN = 0

# Automatically start the service when there are subscribers to its topic(s),
# or when another service requires it.
byte ON_DEMAND = 0

# Always run the service.
#
# Note: the service may still not run if it depends on a disabled service.
byte ALWAYS = 1

# Never run the service,
# even if some other service depends on it.
byte NEVER = 2

byte TRIGGER_MAX = 2
