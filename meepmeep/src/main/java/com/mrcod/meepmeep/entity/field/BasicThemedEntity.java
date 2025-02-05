package com.mrcod.meepmeep.entity.field;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.entity.Entity;

import org.jetbrains.annotations.NotNull;

public abstract class BasicThemedEntity implements Entity {
    private final MeepMeep meepMeep;
    private final String tag;
    private int zIndex = 0;


    public BasicThemedEntity(MeepMeep meepMeep, String tag) {
        this.meepMeep = meepMeep;
        this.tag = tag;
    }

    @NotNull
    @Override
    public MeepMeep getMeepMeep() {
        return meepMeep;
    }

    @Override
    public int getZIndex() {
        return zIndex;
    }

    @Override
    public void setZIndex(int i) {
        this.zIndex = i;
    }

    @NotNull
    @Override
    public String getTag() {
        return this.tag;
    }
}
