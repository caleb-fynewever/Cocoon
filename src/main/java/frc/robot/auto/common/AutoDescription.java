package frc.robot.auto.common;

import java.lang.annotation.*;

@Target(ElementType.TYPE)
public @interface AutoDescription {
    String description() default "No description";
}
