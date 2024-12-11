package frc.robot.auto.common;

import java.lang.annotation.*;

@Retention(RetentionPolicy.RUNTIME) 
public @interface AutoDescription {
    String description() default "No description";
}
