// Import MQTT service
import { MQTTService } from "./mqttService.js";

// Target specific HTML items
const sideMenu = document.querySelector("aside");
const menuBtn = document.querySelector("#menu-btn");
const closeBtn = document.querySelector("#close-btn");
const themeToggler = document.querySelector(".theme-toggler");

const idpot1 = document.getElementById("pot1");
const idpot2 = document.getElementById("pot2");
const idpot3 = document.getElementById("pot3");
const TittlePage = document.getElementById("TittlePage");


var SelectedDevice="?"



let filtre=""
// Holds the background color of all chart
var chartBGColor = getComputedStyle(document.body).getPropertyValue(
  "--chart-background"
);
var chartFontColor = getComputedStyle(document.body).getPropertyValue(
  "--chart-font-color"
);
var chartAxisColor = getComputedStyle(document.body).getPropertyValue(
  "--chart-axis-color"
);

/*
  Event listeners for any HTML click
*/

function getCookie(cName) {
      const name = cName + "=";
      const cDecoded = decodeURIComponent(document.cookie); //to be careful
      const cArr = cDecoded .split('; ');
      let res;
      cArr.forEach(val => {
          if (val.indexOf(name) === 0) res = val.substring(name.length);
      })
      return res;
}

idpot1.addEventListener("click", () => {
document.cookie="SelectedDevice = Yaourt1"         // object
document.cookie="TitleText = Temperature exterieure sur batterie"  
document.cookie= "Name_OMG= OMG_ESP32_LORA" 
document.cookie= "Has_Battery= 1" 
document.cookie= "Has_Humidity = 0"
location.reload()

});

idpot2.addEventListener("click", () => {
document.cookie="SelectedDevice = Yaourt2"           // object
document.cookie="TitleText = Vers le bar: Temperature et humidité" 
document.cookie= "Name_OMG= OMG_ESP32_LORA2" 
document.cookie= "Has_Battery = 0"
document.cookie= "Has_Humidity = 1"
location.reload()

});

idpot3.addEventListener("click", () => {
document.cookie="SelectedDevice = 288AD011000"           // object
document.cookie="TitleText = Vers le Mur Salle à manger " 
document.cookie= "Name_OMG= OMG_ESP32_LORA" 
document.cookie= "Has_Battery = 0"
document.cookie= "Has_Humidity = 0"
location.reload()

});

menuBtn.addEventListener("click", () => {
  sideMenu.style.display = "block";
});

closeBtn.addEventListener("click", () => {
  sideMenu.style.display = "none";
});

themeToggler.addEventListener("click", () => {
  document.body.classList.toggle("dark-theme-variables");
  themeToggler.querySelector("span:nth-child(1)").classList.toggle("active");
  themeToggler.querySelector("span:nth-child(2)").classList.toggle("active");

  // Update Chart background
  chartBGColor = getComputedStyle(document.body).getPropertyValue(
    "--chart-background"
  );
  chartFontColor = getComputedStyle(document.body).getPropertyValue(
    "--chart-font-color"
  );
  chartAxisColor = getComputedStyle(document.body).getPropertyValue(
    "--chart-axis-color"
  );
  updateChartsBackground();
});

/*
  Plotly.js graph and chart setup code
*/
var temperatureHistoryDiv = document.getElementById("temperature-history");
var voltageHistoryDiv = document.getElementById("voltage-history");
var humidityHistoryDiv = document.getElementById("humidity-history");


var temperatureGaugeDiv = document.getElementById("temperature-gauge");
var voltageGaugeDiv = document.getElementById("voltage-gauge");
var batteryGaugeDiv = document.getElementById("battery-gauge");


const historyCharts = [
  temperatureHistoryDiv,
  voltageHistoryDiv,
  humidityHistoryDiv,

];

const gaugeCharts = [
  temperatureGaugeDiv,
  voltageGaugeDiv,
  batteryGaugeDiv,

];

// History Data
var temperatureTrace = {
  x: [1],
  y: [10],

error_y : {   color: 'f44', symmetric: false, array:[10], arrayminus :[5], type:'data', visible:true} ,

	  
  name: "Temperature",
  mode: "lines+markers",
  type: "scatter",
};
var voltageTrace = {
  x: [],
  y: [],
  name: "voltage",
  mode: "lines",
  
  
 //     line: {shape: 'spline'},
  type: 'line'
   ,
   
};
var humidityTrace = {
  x: [],
  y: [],
  name: "humidity",
  mode: "lines",   //+markers",
  type: "line",
};

var temperatureLayout = {
 // autosize: true,

  title: {
    text: "Temperature",
  },
  font: {
    size: 14,
    color: chartFontColor,
    family: "poppins, san-serif",
  },
  colorway: ["#05AD86"],
  margin: { t: 40, b: 120, l: 40, r: 10, pad: 10 },
  plot_bgcolor: chartBGColor,
  paper_bgcolor: chartBGColor,
  xaxis: {
    color: chartAxisColor,
    linecolor: chartAxisColor,
    gridwidth: "2",
    autorange: true,

  },
  yaxis: {
    color: chartAxisColor,
    linecolor: chartAxisColor,
    gridwidth: "2",
    autorange: true,
  },

   
};
var voltageLayout = {
  autosize: true,
  title: {
    text: "voltage",
  },
  font: {
    size: 14,
    color: chartFontColor,
    family: "poppins, san-serif",
  },
  colorway: ["#05AD86"],
  margin: { t: 40, b: 120, l: 50, r: 10, pad: 0 },
  plot_bgcolor: chartBGColor,
  paper_bgcolor: chartBGColor,
  xaxis: {
    color: chartAxisColor,
    linecolor: chartAxisColor,
    gridwidth: "2",
  },
  yaxis: {
    color: chartAxisColor,
    linecolor: chartAxisColor,
  },
};
var humidityLayout = {
  autosize: true,
  title: {
    text: "humidity",
  },
  font: {
    size: 14,
    color: chartFontColor,
    family: "poppins, san-serif",
  },
  colorway: ["#05AD86"],
  margin: { t: 40, b: 120, l: 100, r: 30, pad: 0 },
  plot_bgcolor: chartBGColor,
  paper_bgcolor: chartBGColor,
  xaxis: {
	  
    color: chartAxisColor,
    linecolor: chartAxisColor,
    gridwidth: "2",
  },
  yaxis: {
    color: chartAxisColor,
    linecolor: chartAxisColor,
  },
};




var config = { responsive: true, displayModeBar: false , type: "scatter", };

// Event listener when page is loaded
window.addEventListener("load", (event) => {
  Plotly.newPlot( temperatureHistoryDiv,[temperatureTrace],    temperatureLayout,  config  );
  Plotly.newPlot(voltageHistoryDiv, [voltageTrace], voltageLayout, config);
  Plotly.newPlot(humidityHistoryDiv, [humidityTrace], humidityLayout, config);
  

 var SelectedDevice= getCookie("SelectedDevice" )
 
 if (SelectedDevice==undefined)
	 {
	document.cookie="SelectedDevice = Yaourt1"         // object
	document.cookie= "Name_OMG= OMG_ESP32_LORA" 
	document.cookie="TitleText = Temperature exterieure sur batterie"  
	document.cookie= "Has_Battery= 1" 
	document.cookie= "Has_Humidity= 1" 
	}
 
 else
 { 
   var Name_OMG= getCookie("Name_OMG" )
 
  var Has_Battery= getCookie("Has_Battery" )
  if (Has_Battery==1)
       var val=  "block";
	else
       var val=  "none";
		
  
	const voltage = document.getElementsByClassName("voltage");
 	for (var i=0; i<voltage.length; i++) {
		voltage[i].style.display = val
		}
	
	const voltagehistory = document.getElementById("voltage-history");
	voltagehistory.style.display = val
		



  var Has_Humidity= getCookie("Has_Humidity" )
  if (Has_Humidity==1)
       var val=  "block";
	else
       var val=  "none";
		
	  
	const humidity = document.getElementsByClassName("humidity");
 	for (var i=0; i<humidity.length; i++) {
		humidity[i].style.display = val
		}
	
	const humidityhistory = document.getElementById("humidity-history");
			humidityhistory.style.display = val
	
	
  }
 
 
  // Get MQTT Connection
  fetchMQTTConnection();

  // Run it initially
  
  
  
  var oDiv = document.getElementsByClassName('shiftdateelement');

oDiv[0].addEventListener('click',function (e) {
   changefiltre(-1)
 // alert('left');
  e.stopPropagation();
},true);
oDiv[1].addEventListener('click',function (e) {
  changefiltre(1)
 // alert('right');
  e.stopPropagation();
},true);


  document.body.onclick= function(e){
   e=window.event? event.srcElement: e.target;
   if(e.className && e.className.indexOf('shiftdateelement')!=-1)alert('hohoho');
}

  
  handleDeviceChange(mediaQuery);
});

// Gauge Data
var temperatureData = [
  {
	  
    domain: { x: [0, 1], y: [0, 1] },
    value: 0,
    title: { text: "Temperature" },
    type: "indicator",
    mode: "gauge+number+delta",
    delta: { reference: 30 },
    gauge: {
      axis: { range: [-20, 60] },
      steps: [
	     { range: [-20, 0], color: "red" },
        { range: [0, 18], color: "lightgray" },
        { range: [18, 22], color: "gray" },
		{ range: [22, 60], color: "lightgray" },
      ],
      threshold: {
        line: { color: "red", width: 4 },
        thickness: 0.75,
        value: 20,
      },
    },
  },
];

var voltageData = [
  {
    domain: { x: [0, 1], y: [0, 1] },
    value: 0,
    title: { text: "voltage" },
    type: "indicator",
    mode: "gauge+number+delta",
    delta: { reference: 4000 },
    gauge: {
      axis: { range: [2000, 5000] },
      steps: [
        { range: [0, 3200], color: "red" },
        { range: [3200, 4050], color: "grey" },
	 { range: [4050, 5000], color: "red" },
      ],
      threshold: {
        line: { color: "red", width: 4 },
        thickness: 0.75,
        value: 30,
      },
    },
  },
];

var batteryData = [
  {
    domain: { x: [0, 1], y: [0, 1] },
    value: 0,
    title: { text: "battery" },
    type: "indicator",
    mode: "gauge+number+delta",
    delta: { reference: 70 },
    gauge: {
      axis: { range: [null, 100] },
      steps: [
        { range: [0, 10], color: "red" },
		{ range: [10, 40], color: "orange" },
		{ range: [40, 60], color: "yellow" },
        { range: [60, 100], color: "blue" },
      ],
      threshold: {
        line: { color: "red", width: 4 },
        thickness: 0.75,
        value: 30,
      },
    },
  },
];



var layout = { width: 200, height: 150, margin: { t: 0, b: 0, l: 0, r: 0 } };

Plotly.newPlot(temperatureGaugeDiv, temperatureData, layout);
Plotly.newPlot(voltageGaugeDiv, voltageData, layout);
Plotly.newPlot(batteryGaugeDiv, batteryData, layout);


// Will hold the arrays we receive from our BME280 sensor
// Temperature
let newTempXArray = [];
let newTempYArray = [];
// voltage
let newvoltageXArray = [];
let newvoltageYArray = [];
// battery
let newbatteryXArray = [];
let newbatteryYArray = [];


// The maximum number of data points displayed on our scatter/line graph
let MAX_GRAPH_POINTS = 220;
let ctr = 0;


// Callback function that will retrieve our latest sensor readings and redraw our Gauge with the latest readings
function since() {

var today = new Date();
var time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();

var date1 = new Date("2013/04/09 " +time)
var date2 = new Date("2013/04/09 " + mqttupdated.textContent);

var tmp = (date1 - date2 )/1000
if (tmp > 3600)
	tmp=tmp-3600
console.log( mqttupdated.textContent,time  , tmp )
	mqttupdatedsec.textContent=tmp
  setTimeout(since, 1000); 
  	
}

function updateSensorReadings(jsonResponse) {
  console.log(typeof jsonResponse);
  console.log(jsonResponse);
 
if (jsonResponse.TempCelsius !== undefined  )     // cas dernier message update gauge
  { 
   var TitleText= getCookie("TitleText" )
  TittlePage.innerHTML = TitleText + " (" + jsonResponse.name+")";
  mqttupdated.textContent =jsonResponse.Time.split('/')[1].replaceAll('-',':') 
  
  setTimeout(since, 1000); 
  
  
  let temperature = Number(jsonResponse.TempCelsius).toFixed(2);
  let voltage = Number(jsonResponse.Vbatt).toFixed(2);
   //var val = Object.values(jsonResponse);
   //let battery = Number(val[5] ).toFixed(2);
  let battery = Number(jsonResponse.Charge).toFixed(2);
 
  updateBoxes(temperature, voltage, battery,voltage); //use volatge comm humidi
  updateGauge(temperature, voltage, battery, voltage);
  }
  
  if (jsonResponse.Date !== undefined  )     // cas dernier message update chart
  { 
  var u=jsonResponse.Temp
  var TZ= u.split(',')
  
  var T2=TZ[TZ.length-1];
  var T1=TZ[TZ.length-3];
    
  if (T2 >T1)
  {var divsToShow = document.getElementsByClassName('tendenceup');
   var divsToHide = document.getElementsByClassName('tendencedown');
  }
  else
  {  var divsToHide = document.getElementsByClassName('tendenceup');
     var divsToShow = document.getElementsByClassName('tendencedown');
  }
  if (T2 ==T1) 
  { // var divsToShow = document.getElementsByClassName('tendenceup');
    //  var divsToShow= document.getElementsByClassName('tendencedown') ;
  }
  
  
   for(var i = 0; i < divsToHide.length; i++){
        divsToHide[i].style.visibility = "hidden"; 
    //    divsToHide[i].style.display = "none"; 
    }
	 for(var i = 0; i < divsToShow.length; i++){
        divsToHide[i].style.visibility = "show"; 
     //   divsToHide[i].style.display = "block"; 
    }
	
 
 
  updateMiniMaxi(jsonResponse.Jour_tmin,jsonResponse.Jour_tmax)
  
   burstupdateCharts( jsonResponse.Date, jsonResponse.Temp , jsonResponse.tmin, jsonResponse.tmax, temperatureHistoryDiv);
   burstupdateCharts( jsonResponse.Date, jsonResponse.Vbatt , undefined, undefined,  voltageHistoryDiv);
   burstupdateCharts( jsonResponse.Date, jsonResponse.Vbatt ,undefined, undefined ,humidityHistoryDiv);
       
   // xArray.push(ctr++);

 // var t1= time.split("/")   //12-11-2023/21-34-06
 // var t2=t1[1].split("-")
  
  // xArray.push(t2[0]+":"+t2[1] ); 
  

  }
 
}
 
  
function burstupdateCharts( jdate, jvalue , jtmin , jtmax,grapf) {

if (jvalue== undefined )
	return
	
   var arDate = jdate.replace("[","") 
 //  arDate = arDate.replace("]","") 
//   arDate = arDate.replaceAll("-23","")
 //  arDate = arDate.replaceAll("'","") 

	var arDate=arDate.split(',') 



   var artemp = jvalue.replace("[","") 
   artemp = artemp.replace("]","") 
   var artemperature=artemp.split(',') 
	

   var xArray=[]
   xArray.push (arDate)
   var yArray=[]
   yArray.push(artemperature);
   
   
     var error_y=[]
     var error_y_minus=[]
	 
 if (jtmin !== undefined )
 {
	   error_y=  jtmax.split(',') 
	    error_y_minus=  jtmin.split(',') 
 
 for (let u in error_y)
	 {
	 error_y[u]=error_y[u]- artemperature[u]
	 error_y_minus[u]= artemperature[u]-error_y_minus[u]
		 
	 }
 }

	 

 
  var data_update = {
   x: xArray,
    y: yArray,
	error_y : {color:"F55", symmetric: false, array:error_y,  arrayminus :error_y_minus, type:'data', visible:true} ,


	
  };
  
   Plotly.update(grapf, data_update);
  
}


  // Update Temperature Line Chart
 /* updateCharts(
    temperatureHistoryDiv,
    newTempXArray,
    newTempYArray,
    temperature,  jsonResponse.Time
  );
  */
  /*
  // Update voltage Line Chart
  updateCharts(
    voltageHistoryDiv,
    newvoltageXArray,
    newvoltageYArray,
    voltage , jsonResponse.Time
  );
  // Update battery Line Chart
  updateCharts(
    batteryHistoryDiv,
    newbatteryXArray,
    newbatteryYArray,
    battery ,jsonResponse.Time
  );
*/



function updateMiniMaxi(mini, maxi) {
  let min = document.getElementById("minTempjour");
  let max = document.getElementById("maxTempjour");

  min.innerHTML = mini + " °C";
  max.innerHTML = maxi + " °C";


}


function updateBoxes(temperature, voltage, battery ,humidity) {
  let temperatureDiv = document.getElementById("temperature");
  let voltageDiv = document.getElementById("voltage");
  let batteryDiv = document.getElementById("battery");
  let humidityDiv = document.getElementById("humidity");


  temperatureDiv.innerHTML = temperature + " °C";
  voltageDiv.innerHTML = voltage + " mV";
  batteryDiv.innerHTML = battery + " %";
  humidityDiv.innerHTML = humidity + " %";
}

function updateGauge(temperature, voltage, battery) {
  var temperature_update = {
    value: temperature,
  };
  var voltage_update = {
    value: voltage,
  };
  var battery_update = {
    value: battery,
  };

  Plotly.update(temperatureGaugeDiv, temperature_update);
  Plotly.update(voltageGaugeDiv, voltage_update);
  Plotly.update(batteryGaugeDiv, battery_update);
  
}

function updateCharts(lineChartDiv, xArray, yArray, sensorRead ,time) {
  if (xArray.length >= MAX_GRAPH_POINTS) {
    xArray.shift();
  }
  if (yArray.length >= MAX_GRAPH_POINTS) {
    yArray.shift();
  }
 // xArray.push(ctr++);
  var t1= time.split("/")   //12-11-2023/21-34-06
  var t2=t1[1].split("-")
  
   xArray.push(t2[0]+":"+t2[1] ); 
  yArray.push(sensorRead);

  var data_update = {
	
    x: [xArray],
    y: [yArray],
  };

  Plotly.update(lineChartDiv, data_update);
}

function updateChartsBackground() {
  // updates the background color of historical charts
  var updateHistory = {
    plot_bgcolor: chartBGColor,
    paper_bgcolor: chartBGColor,
    font: {
      color: chartFontColor,
    },
    xaxis: {
      color: chartAxisColor,
      linecolor: chartAxisColor,
    },
    yaxis: {
      color: chartAxisColor,
      linecolor: chartAxisColor,
    },
  };
  historyCharts.forEach((chart) => Plotly.relayout(chart, updateHistory));

  // updates the background color of gauge charts
  var gaugeHistory = {
    plot_bgcolor: chartBGColor,
    paper_bgcolor: chartBGColor,
    font: {
      color: chartFontColor,
    },
    xaxis: {
      color: chartAxisColor,
      linecolor: chartAxisColor,
    },
    yaxis: {
      color: chartAxisColor,
      linecolor: chartAxisColor,
    },
  };
  gaugeCharts.forEach((chart) => Plotly.relayout(chart, gaugeHistory));
}

const mediaQuery = window.matchMedia("(max-width: 400px)");

mediaQuery.addEventListener("change", function (e) {
  handleDeviceChange(e);
});

function handleDeviceChange(e) {
  if (e.matches) {
    console.log("Inside Mobile");
    var updateHistory = {
      width: 400,
      height: 250,
      "xaxis.autorange": true,
      "yaxis.autorange": true,
    };
    historyCharts.forEach((chart) => Plotly.relayout(chart, updateHistory));
  } else {
    var updateHistory = {
      width: 550,
      height: 260,
      "xaxis.autorange": true,
      "yaxis.autorange": true,
    };
    historyCharts.forEach((chart) => Plotly.relayout(chart, updateHistory));
  }
}

/*
  MQTT Message Handling Code
*/
const mqttStatus = document.querySelector(".status");
const mqttupdated = document.querySelector(".updated");
const mqttupdatedsec = document.querySelector(".updatedsec");

  var mqttService 
  
function onConnect(message) {
  mqttStatus.textContent = "Connected";
}
function onMessage(topic, message) {
  var stringResponse = message.toString();
  console.log(message)
  var messageResponse = JSON.parse(stringResponse);
  
  if (messageResponse.hex == undefined  &&  messageResponse.name !== undefined   )
	updateSensorReadings(messageResponse);
  else 
  {console.log("skip:"+topic)
  
  }
  //b=0
 // mqttService.publish(topic,b)
}

function onError(error) {
  console.log(`Error encountered :: ${error}`);
  mqttStatus.textContent = "Error";
}

function onClose() {
  console.log(`MQTT connection closed!`);
  mqttStatus.textContent = "Closed";
}


function padWithLeadingZeros(num, totalLength) {
  return String(num).padStart(totalLength, '0');
}

function fetchMQTTConnection() {
  fetch("env", {
    method: "GET",
    headers: {
      "Content-type": "application/text; charset=UTF-8",
    },
  })
    .then(function (response) {
		console.log("reponse  :", response)
		
 var SelectedDevice= getCookie("SelectedDevice" )
 var Name_OMG= getCookie("Name_OMG" )
 var Has_Battery= getCookie("Has_Battery" )
 
 
	 initializeMQTTConnection("wss://broker.emqx.io:8084/mqtt", "home/"+Name_OMG +"/MERGEtoMQTT/"+ SelectedDevice +"/LastMessage/#");
	 
	 var ladate=new Date()
	 var chd=ladate.getFullYear()+ "-"  + padWithLeadingZeros((ladate.getMonth()+1),2)  +  "-" +  padWithLeadingZeros((ladate.getDate()),2)
	 
 	 initializeMQTTConnection("wss://broker.emqx.io:8084/mqtt", "home/"+ Name_OMG+ "/MERGEtoMQTT/"+SelectedDevice+ "/Histo/"+chd+"/#");
      
  	 let idategraph = document.getElementById('dategraph');
	 var ifiltre= idategraph.innerHTML=chd
  
   
  
	 return response.json();
	  
    })
   
}
function initializeMQTTConnection(mqttServer, mqttTopic) {
  console.log(
    `Initializing connection to :: ${mqttServer}, topic :: ${mqttTopic}`
  );
  var fnCallbacks = { onConnect, onMessage, onError, onClose };

  mqttService = new MQTTService(mqttServer, fnCallbacks);
  mqttService.connect();

  mqttService.subscribe(mqttTopic);
}


// On ajoute au prototype de l'objet Date une méthode personnalisée
Date.prototype.addDays = function(days) {
// On récupère le jour du mois auquel on ajoute le nombre de jour passé en paramètre
var day = this.getDate()+days;
// On définit le jour du mois (This représente la date sur laquelle on effectue l'opération)
this.setDate(day);
}



 function changefiltre(dir)
 {
	 
	 let idategraph = document.getElementById('dategraph');
	 var ifiltre= idategraph.innerHTML
  
  
  
	 var myDate = new Date( ifiltre );
// On additionne le nombre de jour voulu (ex: 2 jours)



if (dir ==-1)	
	{myDate.addDays(-1);	
	
	}

if (dir ==1)
	{myDate.addDays(1);	
	
	}
 

  var data_update = {
   x: [0],
    y: [0]
		
  };
  
   Plotly.update(temperatureHistoryDiv, data_update);
   
   

 var filtre=myDate.getFullYear()+ "-"  + padWithLeadingZeros((myDate.getMonth()+1),2)  +  "-" +  padWithLeadingZeros((myDate.getDate()),2)

var SelectedDevice= getCookie("SelectedDevice" )
 var Name_OMG= getCookie("Name_OMG" )
 
 
 initializeMQTTConnection("wss://broker.emqx.io:8084/mqtt", "home/"+Name_OMG+"/MERGEtoMQTT/"+SelectedDevice+"/Histo/"+filtre+"/#");
 
 	  idategraph.innerHTML=filtre
  
  
 }