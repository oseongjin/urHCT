<?php
$hostname = "localhost";
$username = "robotdb";
$dbpasswd = "rbctrl0927";
$dbname = "rbdatabase";

// Create connection
$conn = mysqli_connect($hostname, $username, $dbpasswd, $dbname);

// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
	error("DBconnect Fail");
	exit;
} 

$sql  = "select * from tempdata order by idx desc";
$rlst = mysqli_query($conn, $sql);
$dcnt = mysqli_num_rows($rlst);
?>
<!doctype html>
<html lang="ko">
    <head>
        <meta charset="UTF-8">
        <title>PM Setup Page</title>
        <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, minimum-scale=1.0, user-scalable=no, target-densitydpi=medium-dpi">
        <meta http-equiv="X-UA-Compatible" content="IE=edge,chrome=1">
        <link rel="stylesheet" href="common/css/total_Style.css">
        <script type="text/javascript" src="common/js/jquery-3.2.1.min.js"></script>
        <script type="text/javascript" src="common/js/menu.js"></script>
        <script src="common/js/raphael.2.1.0.min.js"></script>
        <script src="common/js/justgage.1.0.1.min.js"></script>
        <script>

        </script>
    </head>
    <body>
        <header id="header">
            <h1 class="logo"><a href="#none">HCT 체온측정 로봇 데이터 </a></h1>
            <div class="cd-morph-dropdown" style="display: ;">
                <a href="#0" class="nav-trigger">Open Nav<span aria-hidden="true"></span></a>
                <div class="morph-dropdown-wrapper">
                    <div class="dropdown-list">
                        <ul>
                            <li><a href="#" class="label">Temp Threshold</a></li>
                            <li><a href="" class="label">Network</a></li>
                            <li><a href="" class="label">Robot Config </a></li>
                        </ul>
                    </div> 
                </div> 
            </div>
        </header>
        <div id="container">
            <div id="con_body">
                <article>
                    <section class="clear_box">
                        <table class="tb_base" summary=" Threshold Setting " cellpadding="0">
                            <caption>
                                <span>수집데이터</span>
                            </caption>
                            <colgroup>
                                <col width="5%" />
                                <col width="10%" />
                                <col width="7%" />
                                <col width="7%" />
                                <col width="9%" />
                                <col width="10%" />
                                <col width="13%" />
                                <col width="13%" />
                                <col width="13%" />
                                <col width="13%" />
                            </colgroup>
                            <tbody class="tb_center">
                                <tr>
                                    <th>센서</th>
                                    <th>H04</th>
                                    <th>측정거리</th>
                                    <th>10.5cm</th>
                                    <th>보정 오차</th>
                                    <th>-0.95</th>
                                    <th colspan='4'></th>
                                </tr>
                                <tr>
                                    <th>번호</th>
                                    <th id="time">시간</th>
                                    <th id="dist">측정거리(cm)</th>
                                    <th>편차(cm)</th>
                                    <th id="rawtemp">측정온도(raw)</th>
                                    <th>측정온도(보정)(raw)</th>
                                    <th id="emitemp">환산온도(표준 0.97)</th>
                                    <th>환산온도(보정-표준 0.97)</th>
                                    <th>환산온도(수정 0.88)</th>
                                    <th>환산온도(보정-수정 0.88)</th>
                                </tr>
                                <?php
                                while($result = mysqli_fetch_array($rlst)){
                                ?>
                                <tr>
                                    <th><?php echo $dcnt; ?></th>
                                    <td><?php echo $result[1]; ?></th>
                                    <td><?php echo number_format($result[2]/10, 1); //측정거리 ?></th> 
                                    <td><?php echo number_format((105 - $result[2])/10, 1); //거리 편차 ?></th>
                                    <td><?php echo number_format($result[4]/10000, 1); //측정온도 ?></th>
                                    <td><?php echo number_format(($result[4]/10000) + 0.95, 2); //보정온도 ?></th>
                                    <td><?php echo number_format((($result[4]/10000) / 0.97), 2); //표준환산온도 ?></th>
                                    <td><?php echo number_format((($result[4]/10000) + 0.95) / 0.97, 2); //보정 환산온도 ?></th>
                                    <td><?php echo number_format($result[5]/1000, 1); //환산온도 ?></th>
                                    <td><?php echo number_format($result[5]/1000 + 0.95, 1); //환산온도 보정 ?></th>
                                </tr>
                                <?php
                                    $dcnt = $dcnt - 1;
                                }
                                mysqli_close($conn);
                                ?>
                            </tbody>
                        </table>
                        <!-- <h3 class="cont_title1 mt10"> ADC infomation</h3> -->
                        
                    </section>
                </article>
            </div>
        </div>
        <footer>
        </footer>
    </body>
</html>