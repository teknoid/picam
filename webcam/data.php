
<?php
header('Content-Type: application/json');

$image = '/ram/webcam/current.jpg';
$nexus = '/ram/NEXUS/231/0/';

if (file_exists($nexus) && file_exists($image)) {
    $json = array(
        'mtime' => date('d.m.Y H:m', filemtime($image)),
        'temp' => 'Temp ' . str_replace(PHP_EOL, '', file_get_contents($nexus . 'temp')) . '°C',
        'humi' => 'Hum ' . str_replace(PHP_EOL, '', file_get_contents($nexus . 'humi')) . '%'
    );
    echo json_encode($json);
}
?>
