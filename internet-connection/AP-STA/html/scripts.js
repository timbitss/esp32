async function getTemperature() {
    const result = await fetch("/api/temperature");
    const temperature = await result.json();
    console.log(temperature);
    const el = document.getElementById("temperature-val");
    el.innerText = temperature.Temperature;
  }
  setInterval(getTemperature, 1000);
  
  let LED_switch = false;
  async function toggleLed() {
    const el = document.getElementById("led-button");
    LED_switch = !LED_switch;
    fetch("api/led", { method: "POST", body: JSON.stringify({ LED_switch }) });
    if (isLedOn) {
      el.classList.add("led-on");
      el.classList.remove("led-off");
    } else {
      el.classList.add("led-off");
      el.classList.remove("led-on");
    }
  }