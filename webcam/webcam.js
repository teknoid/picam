function high() {
	window.location.href = "/webcam/h/webcam.html";
}

function low() {
	window.location.href = "/webcam/l/webcam.html";
}

function live(res) {
	var hours = document.querySelectorAll('#navigation-top > ul > li');
	for (var i = 0; i < hours.length; i++) {
		hours[i].style.background = '#fff';
	}
	document.getElementById('video').style.display = "none";
	document.getElementById('navigation-top').style.display = "block";
	document.getElementById("image").style.display = "inline";
	if (res) {
		window.location.href = res;
	} else {
		window.location.href = window.location.href;
	}
}

function picture(hour, src) {
	var hours = document.querySelectorAll('#navigation-top > ul > li');
	for (var i = 0; i < hours.length; i++) {
		hours[i].style.background = '#fff';
	}
	hour.style.background = '#efefef';
	document.getElementById('video').style.display = "none";
	var myImage = document.getElementById("image");
	myImage.style.display = "inline";
	myImage.src = src;
}

function video(vidURL) {
	document.getElementById('image').style.display = "none";
	document.getElementById('navigation-top').style.display = "none";
	var myVideo = document.getElementById('video');
	myVideo.style.display = "inline";
	myVideo.src = vidURL;
	myVideo.load();
	myVideo.play();
}

function loadVideos() {
	var request = new XMLHttpRequest();
	request.open("GET", "../videos.php");
	request.addEventListener('load', function(event) {
		if (this.status == 200) {
			var items = JSON.parse(this.responseText);
			for (var i = 0; i < items.length; i++) {
				var item = items[i];
				var li = document.createElement("li");
				li.appendChild(document.createTextNode(item.name));
				li.setAttribute('onclick', 'video("../videos/' + item.file + '")')
				document.getElementById("videos").appendChild(li);
			}
		}
	});
	request.send();
}

function updateImage() {
	var current = document.getElementById("image");
	if (current.src.indexOf("&ts=") > -1) {
		current.src = current.src.substring(0, current.src.lastIndexOf("&ts="))
				+ "&ts=" + new Date().getTime();
	}
}

function updateData() {
	var request = new XMLHttpRequest();
	request.open("GET", "../data.php");
	request.addEventListener('load', function(event) {
		if (this.status == 200) {
			var data = JSON.parse(this.responseText);
			Object.entries(data).forEach(([k, v]) => {
				var e = document.getElementById(k);
				if (e) e.innerHTML = v;
			});
		}
	});
	request.send();
}

function update() {
	updateImage();
	updateData();
}

window.onload = function() {
	updateData();
	loadVideos();
	setInterval(update, 10000);
}
