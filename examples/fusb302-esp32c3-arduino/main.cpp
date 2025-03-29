#include "pd.h"

//pd::Fusb302Esp32Arduino fusb302;
pd::Sink sink;
pd::DummyDriver driver(sink);
pd::PRL prl(sink, driver);
pd::PE pe(sink, prl, driver);
pd::TC tc(sink, driver);
pd::DPM dpm(sink, pe);


void setup() {

}

void loop() {}
