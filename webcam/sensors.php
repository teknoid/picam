
<?php
header('Content-Type: application/json');

$source = '/ram/433/433.json';

$exclude = array(
    'Acurite-986',
    'Smoke-GS558',
    'Proove-Security',
    'Nexa-Security',
    'Rubicson-Temperature',
    'Nexus-TH',
    'Prologue-TH',
    'Generic-Temperature',
);

function temp_f2c($in)
{
    $out = 5 / 9 * ($in - 32);
    return number_format((float) $out, 0, '.', '');
}

function pressure_kpa2bar($in)
{
    $out = $in * 0.01;
    return number_format((float) $out, 2, '.', '');
}

function pressure_psi2bar($in)
{
    $out = $in * 0.06894757;
    return number_format((float) $out, 2, '.', '');
}

if (file_exists($source)) {
    $out[] = array();
    $handle = fopen($source, "r");
    while (($line = fgets($handle)) !== false) {
        $in = json_decode($line, true);

        $model = $in['model'];
        $id = $in['id'];

        if (empty($model))
            continue;

        if (in_array($model, $exclude))
            continue;

        unset($in['model']);
        unset($in['id']);
        unset($in['channel']);
        unset($in['mic']);

        if (! isset($out[$model][$id]))
            $out[$model][$id] = array();

        foreach ($in as $k => $v) {
            switch ($k) {

                case "temperature_F":
                    $kk = 'temperature_C';
                    if (! isset($out[$model][$id][$kk]))
                        $out[$model][$id][$kk] = array();
                    array_push($out[$model][$id][$kk], temp_f2c($v));
                    break;

                case "pressure_PSI":
                    $kk = 'pressure_BAR';
                    if (! isset($out[$model][$id][$kk]))
                        $out[$model][$id][$kk] = array();
                    array_push($out[$model][$id][$kk], pressure_psi2bar($v));
                    break;

                case "pressure_kPa":
                    $kk = 'pressure_BAR';
                    if (! isset($out[$model][$id][$kk]))
                        $out[$model][$id][$kk] = array();
                    array_push($out[$model][$id][$kk], pressure_kpa2bar($v));
                    break;

                default:
                    if (! isset($out[$model][$id][$k]))
                        $out[$model][$id][$k] = array();
                    array_push($out[$model][$id][$k], $v);
            }
        }
    }
    fclose($handle);

    echo json_encode($out);
}
?>
