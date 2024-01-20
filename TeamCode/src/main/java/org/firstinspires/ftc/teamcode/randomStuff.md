Ok this was some code in RegularTeleOp which I removed but might be useful in auto.
It is supposed to be able to convert cm to ticks for the slides.

```java
/*
    Notes on PID
    position = revolutions * cpr
    revolutions = desired distance / circumference

    position = desired distance * cpr / circumference
*/
public class lol {
    public lol {
        public static final double slidesCPR = 384.5;
        double circumferenceSlides = 3.9; // 3.9 cm, 39mm
        double[] distanceArray = {0.0, 25.0, 50.0, 75.0}; // in cm
        double[] vertSlideArray = {0.0, 0.0, 0.0, 0.0}; // is set in the for loop below
        for (int i = 0; i < 4; i++) {
            vertSlideArray[i] = distanceArray[i] * slidesCPR / circumferenceSlides;
        }
    }
}
```
