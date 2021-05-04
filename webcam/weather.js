function updateData() {
	var request = new XMLHttpRequest();
	request.open("GET", "sensors.php");
	request.addEventListener('load', function(event) {
		if (this.status == 200) {
			var data = JSON.parse(this.responseText);
			Object.entries(data).forEach(([k, v]) => {
				const sensor = document.querySelector('#' + k);
				if (sensor) { 
					const value = sensor.querySelector('.value');
					if (value) value.innerHTML = v;
				}
			});
		}
	});
	request.send();
}

function show(e, interval) {
	var intervals = document.querySelectorAll('#navigation-top > ul > li');
	for (var i = 0; i < intervals.length; i++) {
		intervals[i].style.background = '#fff';
	}
	e.style.background = '#efefef';
	var imgs = document.querySelectorAll('img');
	Object.entries(imgs).forEach(([i, img]) => {
		const token = img.src.replace(/^.*[\\\/]/, '').split('.');
		const newsrc = '/monitorix/imgs/' + token[0] + '.' + interval + '.' + token[2];
		img.src = newsrc;
	});
}

function update() {
	updateData();
}

window.onload = function() {
	updateData();
	setInterval(update, 60000);
}