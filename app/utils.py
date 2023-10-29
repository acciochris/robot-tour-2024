def integral(offset: float = 0.0, initial: float = 0.0, step: float | None = None):
    """Numerical integration"""

    total = offset
    last = initial
    default_step = step

    def accumulate(value: float | None = None, step: float | None = None):
        nonlocal total, last

        # don't change anything if value is not given
        if value is None:
            return total

        # fall back to default step
        if step is None:
            step = default_step

        # error if neither step nor default_step is specified
        if step is None:
            raise ValueError("No step specified")

        total += step * (last + value) / 2
        last = value

        return total

    return accumulate
