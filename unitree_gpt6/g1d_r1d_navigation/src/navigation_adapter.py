"""R1-D preview adapter; implement a separate verified SDK adapter for hardware."""

from unitree_gpt6.g1d_r1d_navigation.common.runtime import PreviewAdapter


class NavigationAdapter(PreviewAdapter):
    """Return preview feedback without connecting to a robot."""
