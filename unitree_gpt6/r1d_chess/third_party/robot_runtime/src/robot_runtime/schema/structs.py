from collections.abc import Sized
from typing import Annotated, Any, ClassVar, TypeVar

import msgspec


T = TypeVar("T", bound="SchemaStruct")


def get_schema_entry_key(value: Any) -> str:
    schema_key = getattr(value, "schema_key", None)
    type_name = type(value).__name__
    return (
        f"{type_name}:{schema_key}"
        if isinstance(schema_key, str) and schema_key
        else type_name
    )


def require_optional_lengths(
    owner: object,
    *fields: tuple[str, Sized | None, int],
) -> None:
    owner_name = (
        owner
        if isinstance(owner, str)
        else owner.__name__
        if isinstance(owner, type)
        else type(owner).__name__
    )
    for field_name, value, expected_length in fields:
        if value is None:
            continue
        length = len(value)
        if length != expected_length:
            raise ValueError(
                f"{owner_name}.{field_name} expected length "
                f"{expected_length}, got {length}."
            )


class SchemaStruct(
    msgspec.Struct,
    omit_defaults=True,
    forbid_unknown_fields=True,
    kw_only=True,
):
    list_fields: ClassVar[frozenset[str]] = frozenset()

    def update(self: T, **fields: Any) -> T:
        known_fields = self.__struct_fields__

        for field_name, value in fields.items():
            if field_name not in known_fields:
                raise AttributeError(
                    f"{type(self).__name__} has no field {field_name!r}."
                )
            if value is not None and field_name in self.list_fields:
                if hasattr(value, "tolist"):
                    value = value.tolist()
                else:
                    value = list(value)
            setattr(self, field_name, value)

        post_init = getattr(self, "__post_init__", None)
        if post_init is not None:
            post_init()
        return self


class SchemaEntry(
    SchemaStruct,
):
    schema_key: Annotated[
        str,
        msgspec.Meta(description="Unique schema data source key."),
    ]

    def __post_init__(self) -> None:
        pass
