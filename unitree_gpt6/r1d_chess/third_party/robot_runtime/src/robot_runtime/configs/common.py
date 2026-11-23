import math
from collections.abc import Mapping


def require_non_empty_str(value, *, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a non-empty string.")
    return value


def require_mapping(value, *, label: str) -> dict[str, object]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{label} must be a mapping.")
    return dict(value)


def require_choice(value, *, choices: frozenset[str], label: str) -> str:
    value = require_non_empty_str(value, label=label)
    if value not in choices:
        known = ", ".join(sorted(choices))
        raise ValueError(f"{label} must be one of {known}; got {value!r}.")
    return value


def require_finite_float(value, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise TypeError(f"{label} must be numeric.")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite.")
    return number


def require_float_range(value, *, label: str) -> tuple[float, float]:
    if not isinstance(value, list | tuple):
        raise TypeError(f"{label} must be a list or tuple.")
    if len(value) != 2:
        raise ValueError(f"{label} must contain lower and upper values.")
    lower = require_finite_float(value[0], label=f"{label}[0]")
    upper = require_finite_float(value[1], label=f"{label}[1]")
    if lower >= upper:
        raise ValueError(f"{label} lower limit must be below upper.")
    return lower, upper


def require_positive_float(value, *, label: str) -> float:
    number = require_finite_float(value, label=label)
    if number <= 0.0:
        raise ValueError(f"{label} must be positive.")
    return number


def require_positive_int(value, *, label: str) -> int:
    if type(value) is not int:
        raise TypeError(f"{label} must be an int.")
    if value <= 0:
        raise ValueError(f"{label} must be positive.")
    return value


def add_unique(
    mapping: dict[str, object],
    key: str,
    value: object,
    *,
    label: str,
) -> None:
    if key in mapping:
        raise ValueError(f"{label} contains duplicate key {key!r}.")
    mapping[key] = value


def reject_unknown_fields(
    cfg: Mapping[str, object],
    allowed_fields: frozenset[str],
    *,
    label: str,
) -> None:
    unknown_fields = tuple(sorted(set(cfg) - allowed_fields))
    if unknown_fields:
        raise KeyError(
            f"{label} contains unsupported fields {unknown_fields!r}."
        )


def normalize_schema_resources(
    resources: object,
    *,
    label: str,
) -> dict[str, dict[str, object]]:
    """Validate a resource list and return resources keyed by schema key."""
    if not isinstance(resources, list):
        raise TypeError(f"{label} must be a list.")

    resources_by_schema_key = {}
    for index, resource in enumerate(resources):
        resource_label = f"{label}[{index}]"
        resource_config = require_mapping(resource, label=resource_label)
        if "schema_key" not in resource_config:
            raise KeyError(f"{resource_label} must define schema_key.")
        schema_key = require_non_empty_str(
            resource_config["schema_key"],
            label=f"{resource_label}.schema_key",
        )
        if schema_key in resources_by_schema_key:
            raise ValueError(
                f"{label} contains duplicate schema key {schema_key!r}."
            )
        resources_by_schema_key[schema_key] = resource_config
    return resources_by_schema_key
