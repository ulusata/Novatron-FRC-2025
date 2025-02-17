package frc.lib.abstracts;

enum LedModeState {
    UNINITIALIZED, INITIALIZED, UPDATE
}

public abstract class BaseLedMode {
    protected LedModeState state = LedModeState.UNINITIALIZED;
    protected final String name;

    public BaseLedMode(String name) {
        if (name == null) {
            throw new IllegalArgumentException("Name cannot be null");
        }
        this.name = name;
    }

    public void enable() {
        if (state == LedModeState.UNINITIALIZED) {
            enableEffect();
            state = LedModeState.INITIALIZED;
        }
    }

    public void update() {
        if (state == LedModeState.UNINITIALIZED) {
            enable();
        }
        if (state == LedModeState.INITIALIZED || state == LedModeState.UPDATE) {
            updateEffect();
            state = LedModeState.UPDATE;
        }
    }

    public void disable() {
        if (state == LedModeState.UPDATE) {
            disableEffect();
            state = LedModeState.UNINITIALIZED;
        }
    }

    public String getName() {
        return name;
    }

    /*
     * This function is called once when the mode is first UPDATE
     */
    protected abstract void enableEffect();

    /*
     * This function is called every time the mode is UPDATE
     */
    protected abstract void updateEffect();

    /*
     * This function is called every time the mode is disabled
     */
    protected abstract void disableEffect();
}