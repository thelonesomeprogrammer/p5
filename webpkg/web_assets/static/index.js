async function fetchData() {
  try {
    const response = await fetch("/data");
    const text = await response.text();
    const data = JSON.parse(text);

    data.forEach((value, index) => {
      const station = document.getElementById(`station-${index + 1}`);
      station.innerHTML = `
                    Station ID: ${value.Stationid}<br>
                    Carrier ID: ${value.last_carrierid}<br>
                    Read Time: <br> 
                    ${value.read_time}`;
    });
  } catch (error) {
    console.error("Error fetching data:", error);
  }
}
fetchData();
setInterval(fetchData, 500); // Refresh every 5 seconds
