package com.mrcod.meepmeep;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;

public class Main {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meep = new MeepMeep(720);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);
        ColorScheme scheme = new ColorSchemeBlueDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);

        meep.start();
    }
}
