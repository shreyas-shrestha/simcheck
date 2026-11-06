# SimCheck task

Write `solution.py` implementing `make_estimator(public_spec)`, `Estimator.step(new_measurements)` and
`choose_route(public_spec)` as described in the contract. Use GTSAM. Read `spec.json` for every parameter.

Conventions:
- `initial_pose.cov_diag_variances` are VARIANCES. Every `*_noise_std` entry is a STANDARD DEVIATION in SI
  units (metres, radians), regardless of `input_units`.
- Odometry noise is Pose2 tangent-space noise on the relative pose (x_{k-1} -> x_k), body frame.
- Landmark bearing is measured relative to the robot heading. GPS gives world-frame x, y.
- Returned covariance is 3x3 in the local tangent frame of the returned pose (GTSAM convention).

Scenario-specific notes:
- None.


## spec.json
```json
{
  "units_out": {
    "position": "m",
    "angle": "rad"
  },
  "dt": 0.5,
  "pose_indexing": "N poses x_0..x_{N-1}, N-1 odometry transitions",
  "initial_pose": {
    "mean": [
      0.0,
      0.0,
      0.0
    ],
    "cov_diag_variances": [
      0.01,
      0.01,
      0.001
    ]
  },
  "initial_truth_rule": "true x_0 is sampled from N(mean, diag(cov_diag_variances))",
  "landmarks": {
    "0": [
      1.2,
      5.5
    ],
    "1": [
      3.12,
      5.5
    ],
    "2": [
      5.04,
      5.5
    ],
    "3": [
      6.96,
      5.5
    ],
    "4": [
      8.88,
      5.5
    ],
    "5": [
      10.8,
      5.5
    ]
  },
  "landmark_visibility": {
    "max_range": 5.0,
    "rule": "observed every step when within max_range, no occlusion"
  },
  "sensor_noise_std": {
    "odom": [
      0.02,
      0.02,
      0.01
    ],
    "range": 0.1,
    "bearing": 0.03,
    "gps": 0.15
  },
  "actuator_noise_std": {
    "v": 0.05,
    "omega": 0.05
  },
  "gps_schedule": {
    "rate_steps": 1,
    "dropout_steps": [
      20,
      35
    ]
  },
  "input_units": {
    "odom": "m",
    "bearing": "rad"
  },
  "odom_frame": "body",
  "controller": {
    "type": "pure_pursuit",
    "lookahead": 0.8,
    "v": 0.5,
    "omega_max": 1.0
  },
  "max_steps": 120,
  "arrival_tolerance": 0.3,
  "routes": {
    "A": [
      [
        0.0,
        0.0
      ],
      [
        12.0,
        0.0
      ]
    ],
    "B": [
      [
        0.0,
        0.0
      ],
      [
        2.4000000000000004,
        3.0
      ],
      [
        9.600000000000001,
        3.0
      ],
      [
        12.0,
        0.0
      ]
    ]
  },
  "robot_radius": 0.2,
  "walls": [
    [
      5.4,
      0.45,
      9.0,
      0.45
    ],
    [
      5.4,
      -0.45,
      9.0,
      -0.45
    ]
  ],
  "success_definition": "reach within arrival_tolerance of route end within max_steps with zero collisions",
  "objective": "maximize collision-free arrival probability; distance breaks close ties"
}
```

## measurements.json (one example episode; your estimator is graded on many others)
```json
[{"step": 0, "t": 0.0, "odom": null, "gps": {"x": 0.13663820571183286, "y": 0.1343708992580851}, "landmarks": []}, {"step": 1, "t": 0.5, "odom": {"dx": 0.21640307377794948, "dy": 0.059559631003158, "dtheta": 0.4886025372423295}, "gps": {"x": 0.18842822473705073, "y": 0.31429933136440874}, "landmarks": []}, {"step": 2, "t": 1.0, "odom": {"dx": 0.217545008846335, "dy": 0.05643189171546602, "dtheta": 0.3766346508481456}, "gps": {"x": 0.42450313153339486, "y": 0.34348407004872905}, "landmarks": []}, {"step": 3, "t": 1.5, "odom": {"dx": 0.2270974756173022, "dy": 0.02593173228203815, "dtheta": 0.20184937466053937}, "gps": {"x": 0.6478782800937313, "y": 0.46327206519939385}, "landmarks": []}, {"step": 4, "t": 2.0, "odom": {"dx": 0.18543273848013794, "dy": 0.02147294001119913, "dtheta": 0.06436396310085864}, "gps": {"x": 0.8539767556046849, "y": 0.5813639999964195}, "landmarks": [{"id": "0", "range": 5.110360791131293, "bearing": 0.34209649389938734}]}, {"step": 5, "t": 2.5, "odom": {"dx": 0.21406636227251327, "dy": 0.00016461387480697, "dtheta": -0.047912010269036874}, "gps": {"x": 0.8017303453682859, "y": 0.9252003537596662}, "landmarks": [{"id": "0", "range": 4.9047422872793485, "bearing": 0.40176960010492824}]}, {"step": 6, "t": 3.0, "odom": {"dx": 0.2665084805751088, "dy": 0.023813984368249188, "dtheta": -0.021243083356897816}, "gps": {"x": 0.8319741316746647, "y": 0.9285662469924532}, "landmarks": [{"id": "0", "range": 4.577895257525972, "bearing": 0.5068719521672924}]}, {"step": 7, "t": 3.5, "odom": {"dx": 0.2853138631292763, "dy": -0.017945895184682537, "dtheta": -0.09997408017050695}, "gps": {"x": 1.2439247447543322, "y": 1.1558765796115316}, "landmarks": [{"id": "0", "range": 4.381105622556441, "bearing": 0.595504982368527}, {"id": "1", "range": 4.953023841956641, "bearing": 0.14897369840430708}]}, {"step": 8, "t": 4.0, "odom": {"dx": 0.24139980602078484, "dy": -0.01898423859456941, "dtheta": 0.00832135016571419}, "gps": {"x": 1.2712952301276197, "y": 1.5147646713936274}, "landmarks": [{"id": "0", "range": 3.9860117964248154, "bearing": 0.6233375241586474}, {"id": "1", "range": 4.342048118919046, "bearing": 0.22003084633107806}]}, {"step": 9, "t": 4.5, "odom": {"dx": 0.25349302672588875, "dy": -0.007457190695313472, "dtheta": 0.017503459848943363}, "gps": {"x": 1.5200903829036054, "y": 1.631098645699427}, "landmarks": [{"id": "0", "range": 3.8442782983974713, "bearing": 0.6561909748946464}, {"id": "1", "range": 4.2635580416917565, "bearing": 0.22565500081969248}]}, {"step": 10, "t": 5.0, "odom": {"dx": 0.23325090035613621, "dy": 0.0005927966003238432, "dtheta": -0.057166262686729724}, "gps": {"x": 1.287475676147435, "y": 1.58873828797331}, "landmarks": [{"id": "0", "range": 3.7937869142652096, "bearing": 0.6491856220999344}, {"id": "1", "range": 3.9359625105698504, "bearing": 0.30182431337813087}]}, {"step": 11, "t": 5.5, "odom": {"dx": 0.248359579259954, "dy": -0.01504653183794382, "dtheta": 0.013040317958357382}, "gps": {"x": 1.8574160402927269, "y": 1.8655241196220853}, "landmarks": [{"id": "0", "range": 3.7071354463145023, "bearing": 0.7394520933283504}, {"id": "1", "range": 3.797896694715755, "bearing": 0.20037345441275534}, {"id": "2", "range": 4.954086627710717, "bearing": -0.16786642477152558}]}, {"step": 12, "t": 6.0, "odom": {"dx": 0.2031977033320157, "dy": 0.0230189036889416, "dtheta": 0.007115274937243212}, "gps": {"x": 1.3855240432742, "y": 2.1682395454899512}, "landmarks": [{"id": "0", "range": 3.3180228705621704, "bearing": 0.7842378802829035}, {"id": "1", "range": 3.4320434781898466, "bearing": 0.2630183591323094}, {"id": "2", "range": 4.828626878419957, "bearing": -0.14609693357658093}]}, {"step": 13, "t": 6.5, "odom": {"dx": 0.28556153195046907, "dy": -0.014323721723801383, "dtheta": -0.017047931706411668}, "gps": {"x": 1.8876449824162365, "y": 2.5141537221092434}, "landmarks": [{"id": "0", "range": 3.3148514173190073, "bearing": 0.8321736719507395}, {"id": "1", "range": 3.3812018270123216, "bearing": 0.3121003982656547}, {"id": "2", "range": 4.500672530555434, "bearing": -0.14101336846673052}]}, {"step": 14, "t": 7.0, "odom": {"dx": 0.26032323949645897, "dy": 0.026838804036254223, "dtheta": -0.0014011700279694562}, "gps": {"x": 1.7971128244343983, "y": 2.604654583970136}, "landmarks": [{"id": "0", "range": 3.2218850122479012, "bearing": 1.031543091981355}, {"id": "1", "range": 3.088084115524033, "bearing": 0.3425721621180351}, {"id": "2", "range": 4.13719937811221, "bearing": -0.07154400604028321}]}, {"step": 15, "t": 7.5, "odom": {"dx": 0.2826271022109707, "dy": -0.022958995042837948, "dtheta": -0.144181226142696}, "gps": {"x": 1.9696857480763401, "y": 3.013733489414414}, "landmarks": [{"id": "0", "range": 2.944255716086462, "bearing": 1.1813261477025128}, {"id": "1", "range": 2.8373579470129404, "bearing": 0.5011521670286223}, {"id": "2", "range": 4.015347039552876, "bearing": 0.015973882864817046}]}, {"step": 16, "t": 8.0, "odom": {"dx": 0.29727328954621374, "dy": -0.05965191884449461, "dtheta": -0.28155246550421653}, "gps": {"x": 2.317155673977116, "y": 2.8365419696077567}, "landmarks": [{"id": "0", "range": 2.8536367566825263, "bearing": 1.5521340002577375}, {"id": "1", "range": 2.7963536183670437, "bearing": 0.9213917968853806}, {"id": "2", "range": 3.634489704105418, "bearing": 0.3275802989497252}]}, {"step": 17, "t": 8.5, "odom": {"dx": 0.22157681438206173, "dy": -0.012621647740679493, "dtheta": -0.21204895089148484}, "gps": {"x": 2.626533667347419, "y": 3.0015923970162732}, "landmarks": [{"id": "0", "range": 2.805282482556332, "bearing": 1.795416637098631}, {"id": "1", "range": 2.5934319354676143, "bearing": 1.1278413275496586}, {"id": "2", "range": 3.5992011111613036, "bearing": 0.5317490848606088}, {"id": "3", "range": 4.872875979700818, "bearing": 0.2766464652781875}]}, {"step": 18, "t": 9.0, "odom": {"dx": 0.27565149529702093, "dy": -0.037070842749163196, "dtheta": -0.1874706084565553}, "gps": {"x": 3.3393525092720404, "y": 2.76233562817205}, "landmarks": [{"id": "0", "range": 3.0335967984891554, "bearing": 2.1371934071066736}, {"id": "1", "range": 2.4348043872413134, "bearing": 1.4241911516271788}, {"id": "2", "range": 3.1786498834241925, "bearing": 0.7572251877062941}, {"id": "3", "range": 4.90793767348058, "bearing": 0.5159335545066615}]}, {"step": 19, "t": 9.5, "odom": {"dx": 0.26532041956947483, "dy": -0.013470207593736058, "dtheta": -0.10826510161616905}, "gps": {"x": 3.2751366089272778, "y": 3.3424169327142663}, "landmarks": [{"id": "0", "range": 3.1682900618968413, "bearing": 2.255613796101099}, {"id": "1", "range": 2.5129543750607533, "bearing": 1.619507588145904}, {"id": "2", "range": 2.9477412174702584, "bearing": 0.9428562795519024}, {"id": "3", "range": 4.444315284058338, "bearing": 0.5102239523977858}]}, {"step": 20, "t": 10.0, "odom": {"dx": 0.24722035976617882, "dy": -0.014276507588035107, "dtheta": 0.0005819721211717758}, "gps": null, "landmarks": [{"id": "0", "range": 3.403699827723926, "bearing": 2.2614851120322452}, {"id": "1", "range": 2.4097365098842394, "bearing": 1.671156396593961}, {"id": "2", "range": 2.9670454697335864, "bearing": 0.972448831370115}, {"id": "3", "range": 4.199744791171284, "bearing": 0.6129918577643534}]}, {"step": 21, "t": 10.5, "odom": {"dx": 0.24946489457709536, "dy": -0.00015634171702163732, "dtheta": -0.03374805736890688}, "gps": null, "landmarks": [{"id": "0", "range": 3.4044655508389154, "bearing": 2.358655581092113}, {"id": "1", "range": 2.373469432769761, "bearing": 1.8519458226435006}, {"id": "2", "range": 2.8947358383047153, "bearing": 1.1278492364314765}, {"id": "3", "range": 3.99696532637932, "bearing": 0.6763317761343957}]}, {"step": 22, "t": 11.0, "odom": {"dx": 0.20414893829599864, "dy": 0.0005303792931219526, "dtheta": -0.012557332040064177}, "gps": null, "landmarks": [{"id": "0", "range": 3.724851435246815, "bearing": 2.4969803067586973}, {"id": "1", "range": 2.4614116859515756, "bearing": 1.946270161564585}, {"id": "2", "range": 2.590912773874494, "bearing": 1.162772404498038}, {"id": "3", "range": 3.8940834893070897, "bearing": 0.7460020572760757}]}, {"step": 23, "t": 11.5, "odom": {"dx": 0.2787915538381812, "dy": -0.024134155730181243, "dtheta": 0.026121449140689975}, "gps": null, "landmarks": [{"id": "0", "range": 4.076357101910881, "bearing": 2.4104420267764652}, {"id": "1", "range": 2.653646923794362, "bearing": 2.0107709788434036}, {"id": "2", "range": 2.6296514374656237, "bearing": 1.2693210577982619}, {"id": "3", "range": 3.6355871222590577, "bearing": 0.7495961226030445}]}, {"step": 24, "t": 12.0, "odom": {"dx": 0.29392468495206114, "dy": -0.02748591618104597, "dtheta": 0.01570479103224044}, "gps": null, "landmarks": [{"id": "0", "range": 4.012201951631167, "bearing": 2.4831623547938433}, {"id": "1", "range": 2.7570632775678816, "bearing": 2.1408966921912915}, {"id": "2", "range": 2.331029848625965, "bearing": 1.3823192995745988}, {"id": "3", "range": 3.362826143585098, "bearing": 0.6984926127353934}]}, {"step": 25, "t": 12.5, "odom": {"dx": 0.24065291340275025, "dy": 0.004937815895373218, "dtheta": -0.03546465289036823}, "gps": null, "landmarks": [{"id": "0", "range": 4.28978238976477, "bearing": 2.5342622771789447}, {"id": "1", "range": 2.8354161682965735, "bearing": 2.159393191954381}, {"id": "2", "range": 2.451880037431971, "bearing": 1.4952305178858705}, {"id": "3", "range": 3.3129358843751557, "bearing": 0.9027841491192161}, {"id": "4", "range": 4.588363824999136, "bearing": 0.5576119326523261}]}, {"step": 26, "t": 13.0, "odom": {"dx": 0.24728060275770145, "dy": 0.00920026831147802, "dtheta": -0.017375696505819764}, "gps": null, "landmarks": [{"id": "0", "range": 4.438507413363202, "bearing": 2.529819509537562}, {"id": "1", "range": 3.2184623484507418, "bearing": 2.2445522858438522}, {"id": "2", "range": 2.42354902758662, "bearing": 1.5832111320859124}, {"id": "3", "range": 3.2680638057321816, "bearing": 0.9695717885737523}, {"id": "4", "range": 4.685387382354941, "bearing": 0.5696273912399645}]}, {"step": 27, "t": 13.5, "odom": {"dx": 0.2764047973197916, "dy": 0.027063512634430506, "dtheta": 0.044778858765874084}, "gps": null, "landmarks": [{"id": "0", "range": 4.660097273376866, "bearing": 2.604350477023731}, {"id": "1", "range": 3.2782280753107917, "bearing": 2.289443110490523}, {"id": "2", "range": 2.708900918022643, "bearing": 1.6038085772217014}, {"id": "3", "range": 3.1021850040473993, "bearing": 0.9077560282239752}, {"id": "4", "range": 4.434275928740035, "bearing": 0.6076014968929542}]}, {"step": 28, "t": 14.0, "odom": {"dx": 0.28668905866861943, "dy": -0.013863118475994179, "dtheta": -0.04856830749462169}, "gps": null, "landmarks": [{"id": "1", "range": 3.492012442178008, "bearing": 2.37693856333847}, {"id": "2", "range": 2.5984860426816314, "bearing": 1.8163490623244645}, {"id": "3", "range": 2.8680587447996757, "bearing": 1.1522483133507713}, {"id": "4", "range": 4.1954525436592665, "bearing": 0.7463468617462774}]}, {"step": 29, "t": 14.5, "odom": {"dx": 0.22681732647844396, "dy": -0.00031938055195913093, "dtheta": 0.026931397131921855}, "gps": null, "landmarks": [{"id": "1", "range": 3.586544064162999, "bearing": 2.414634962408808}, {"id": "2", "range": 2.6087307316176007, "bearing": 1.9364426395934027}, {"id": "3", "range": 2.772772649033576, "bearing": 1.217518101375684}, {"id": "4", "range": 3.935203845414711, "bearing": 0.6706556306215194}]}, {"step": 30, "t": 15.0, "odom": {"dx": 0.2210536672720894, "dy": 0.010868839430869831, "dtheta": 0.036581841640107804}, "gps": null, "landmarks": [{"id": "1", "range": 4.02975057891743, "bearing": 2.400139318694368}, {"id": "2", "range": 2.577469782921034, "bearing": 1.899490600199968}, {"id": "3", "range": 2.745366569737069, "bearing": 1.2034757704583416}, {"id": "4", "range": 3.8727045644553475, "bearing": 0.7146635900658262}]}, {"step": 31, "t": 15.5, "odom": {"dx": 0.248455822473009, "dy": -0.0028892588361147543, "dtheta": 0.0052345823210797416}, "gps": null, "landmarks": [{"id": "1", "range": 4.001254734652165, "bearing": 2.458190296580206}, {"id": "2", "range": 2.9070640876724942, "bearing": 1.9858798640541249}, {"id": "3", "range": 2.6297246769633764, "bearing": 1.177663878550943}, {"id": "4", "range": 3.533831266056678, "bearing": 0.7225691232882872}]}, {"step": 32, "t": 16.0, "odom": {"dx": 0.23418878132935533, "dy": -0.012684394321679676, "dtheta": -0.043711644100941124}, "gps": null, "landmarks": [{"id": "1", "range": 4.252351797642064, "bearing": 2.458999565558379}, {"id": "2", "range": 2.8168545177772297, "bearing": 2.0921159293851446}, {"id": "3", "range": 2.3583120361641337, "bearing": 1.4185184089723917}, {"id": "4", "range": 3.454011706281072, "bearing": 0.824432720103287}, {"id": "5", "range": 5.004344176876963, "bearing": 0.49456061391951867}]}, {"step": 33, "t": 16.5, "odom": {"dx": 0.2073919485377766, "dy": -0.060902508170888284, "dtheta": -0.04652224447199079}, "gps": null, "landmarks": [{"id": "1", "range": 4.411319544221436, "bearing": 2.5529105875431615}, {"id": "2", "range": 2.8260258048158415, "bearing": 2.2415987026934374}, {"id": "3", "range": 2.5526747678518262, "bearing": 1.5270005318681656}, {"id": "4", "range": 3.1115155222080957, "bearing": 0.8807789692211898}, {"id": "5", "range": 4.6532748731179, "bearing": 0.5500853219039139}]}, {"step": 34, "t": 17.0, "odom": {"dx": 0.23897673169562547, "dy": 0.00973842416978339, "dtheta": 0.0199650721723089}, "gps": null, "landmarks": [{"id": "1", "range": 4.702708326551782, "bearing": 2.626946520334153}, {"id": "2", "range": 3.197515822481404, "bearing": 2.277577652412363}, {"id": "3", "range": 2.3669698098503194, "bearing": 1.5498881771879462}, {"id": "4", "range": 3.1750461448047846, "bearing": 0.923833633318309}, {"id": "5", "range": 4.511861372523382, "bearing": 0.6046127942218265}]}, {"step": 35, "t": 17.5, "odom": {"dx": 0.2560534050125987, "dy": -0.02851418161027209, "dtheta": 0.0256032260784932}, "gps": null, "landmarks": [{"id": "1", "range": 4.918995648867486, "bearing": 2.5409125475587313}, {"id": "2", "range": 3.2579670976767545, "bearing": 2.245823386101492}, {"id": "3", "range": 2.510884479687322, "bearing": 1.6479565739681004}, {"id": "4", "range": 2.9802276651980617, "bearing": 0.9926668656401887}, {"id": "5", "range": 4.4688081892424645, "bearing": 0.5686461094734176}]}, {"step": 36, "t": 18.0, "odom": {"dx": 0.32868595231336717, "dy": 0.016482217752745176, "dtheta": -0.048591477637486585}, "gps": {"x": 7.418973779445276, "y": 2.876404374873817}, "landmarks": [{"id": "2", "range": 3.6920010518193993, "bearing": 2.38917496432113}, {"id": "3", "range": 2.5155449870228024, "bearing": 1.7701498335941652}, {"id": "4", "range": 2.870538132882807, "bearing": 1.0409505656464662}, {"id": "5", "range": 4.165342134402678, "bearing": 0.6531792765577253}]}, {"step": 37, "t": 18.5, "odom": {"dx": 0.23816074864743017, "dy": -0.030168423808396877, "dtheta": 0.02606213499923244}, "gps": {"x": 7.836926418834074, "y": 2.960019588018122}, "landmarks": [{"id": "2", "range": 3.8078279798218895, "bearing": 2.367794731923297}, {"id": "3", "range": 2.385315707122018, "bearing": 1.8487190068193105}, {"id": "4", "range": 2.627675212663021, "bearing": 1.1457363652444688}, {"id": "5", "range": 3.9827923985109774, "bearing": 0.695918451516393}]}, {"step": 38, "t": 19.0, "odom": {"dx": 0.29868184844177176, "dy": -0.014830055095765036, "dtheta": 0.010663099656742371}, "gps": {"x": 8.076287197316518, "y": 2.9064006336435315}, "landmarks": [{"id": "2", "range": 3.8615895722302365, "bearing": 2.440216579643196}, {"id": "3", "range": 2.584196895396904, "bearing": 1.9592462171199907}, {"id": "4", "range": 2.6811334976336902, "bearing": 1.225149994608726}, {"id": "5", "range": 3.7604414094101712, "bearing": 0.6929797315316009}]}, {"step": 39, "t": 19.5, "odom": {"dx": 0.24830950603059218, "dy": 0.0351509498531715, "dtheta": 0.0060400913651236865}, "gps": {"x": 8.368867040143815, "y": 2.96902444426578}, "landmarks": [{"id": "2", "range": 4.077475990057113, "bearing": 2.455993860681124}, {"id": "3", "range": 2.8382203555294474, "bearing": 2.010344284297731}, {"id": "4", "range": 2.4099743499735564, "bearing": 1.3086221739575414}, {"id": "5", "range": 3.50129892894011, "bearing": 0.7498399117111001}]}, {"step": 40, "t": 20.0, "odom": {"dx": 0.2333766229147251, "dy": -0.005390476030513906, "dtheta": 0.02668192618661623}, "gps": {"x": 8.415940990502019, "y": 2.947993722555002}, "landmarks": [{"id": "2", "range": 4.241046035829062, "bearing": 2.4331838485022317}, {"id": "3", "range": 2.8745619717980704, "bearing": 2.0587302820034843}, {"id": "4", "range": 2.428919874699211, "bearing": 1.3755363712296838}, {"id": "5", "range": 3.3789750592736967, "bearing": 0.7408030113337079}]}, {"step": 41, "t": 20.5, "odom": {"dx": 0.2753090377892652, "dy": -0.015178036261365796, "dtheta": -0.04369722573837817}, "gps": {"x": 8.769090554239739, "y": 3.090144857489436}, "landmarks": [{"id": "2", "range": 4.5083150158770495, "bearing": 2.5651823922118844}, {"id": "3", "range": 3.0363185923381106, "bearing": 2.1733895089045454}, {"id": "4", "range": 2.4681423262934183, "bearing": 1.5118659863889103}, {"id": "5", "range": 3.1599870683159828, "bearing": 0.8533798333623364}]}, {"step": 42, "t": 21.0, "odom": {"dx": 0.2645469267890417, "dy": 0.015452115375311791, "dtheta": -0.0068402157563811575}, "gps": {"x": 8.889036353045322, "y": 2.9942007626063623}, "landmarks": [{"id": "2", "range": 4.713765793617924, "bearing": 2.5744331884432947}, {"id": "3", "range": 3.2611733651702934, "bearing": 2.225297040277688}, {"id": "4", "range": 2.6101625810305644, "bearing": 1.6194543320054686}, {"id": "5", "range": 3.085842518738431, "bearing": 0.944239673253982}]}, {"step": 43, "t": 21.5, "odom": {"dx": 0.23364932261203863, "dy": 0.0054928905010615744, "dtheta": -0.16555983225960982}, "gps": {"x": 9.133500501068445, "y": 3.061638640140019}, "landmarks": [{"id": "2", "range": 4.74436439466199, "bearing": 2.7250375979590498}, {"id": "3", "range": 3.2273724956347185, "bearing": 2.507714233725922}, {"id": "4", "range": 2.5114017507080133, "bearing": 1.7965931055436126}, {"id": "5", "range": 3.0373397173619896, "bearing": 1.1476502785105573}]}, {"step": 44, "t": 22.0, "odom": {"dx": 0.26139289886170475, "dy": -0.03686421730132198, "dtheta": -0.1642576682005381}, "gps": {"x": 9.63421480719102, "y": 2.93831241272252}, "landmarks": [{"id": "3", "range": 3.365738446619322, "bearing": 2.6902148549691276}, {"id": "4", "range": 2.7902456312433643, "bearing": 2.1325744508462723}, {"id": "5", "range": 2.8041490237580415, "bearing": 1.3794307347437824}]}, {"step": 45, "t": 22.5, "odom": {"dx": 0.22530319984881356, "dy": -0.06547500296089168, "dtheta": -0.2509842558697724}, "gps": {"x": 9.542238318801305, "y": 2.982440592138275}, "landmarks": [{"id": "3", "range": 3.811793842044309, "bearing": 2.9434662967506835}, {"id": "4", "range": 2.8358544637526313, "bearing": 2.4367469034785696}, {"id": "5", "range": 3.121395550188959, "bearing": 1.7254414708041423}]}, {"step": 46, "t": 23.0, "odom": {"dx": 0.28024578370036735, "dy": -0.010941655010782812, "dtheta": -0.09185223248575022}, "gps": {"x": 9.871892601691192, "y": 2.812398123734166}, "landmarks": [{"id": "3", "range": 3.9750233042650485, "bearing": 3.0583568107099053}, {"id": "4", "range": 2.947047128358335, "bearing": 2.6002008845914224}, {"id": "5", "range": 2.9948213422662477, "bearing": 1.8942974848088916}]}, {"step": 47, "t": 23.5, "odom": {"dx": 0.22437351304707537, "dy": -0.020623232275335537, "dtheta": -0.14708765237079674}, "gps": {"x": 10.088183336975103, "y": 2.5424346270421734}, "landmarks": [{"id": "3", "range": 4.187847256258201, "bearing": -3.1227642251070384}, {"id": "4", "range": 3.21598864316744, "bearing": 2.7253990620083997}, {"id": "5", "range": 3.141050525894102, "bearing": 2.174955950808889}]}, {"step": 48, "t": 24.0, "odom": {"dx": 0.18408000957200923, "dy": -0.010642807074925785, "dtheta": -0.0396207893848193}, "gps": {"x": 10.199344965869184, "y": 2.143824982868909}, "landmarks": [{"id": "3", "range": 4.5293732888562355, "bearing": -3.0782604473726187}, {"id": "4", "range": 3.587135000773415, "bearing": 2.793853772184848}, {"id": "5", "range": 3.303975274882377, "bearing": 2.2044557551845845}]}, {"step": 49, "t": 24.5, "odom": {"dx": 0.24006443968434082, "dy": 0.016487454229412984, "dtheta": -0.10080461597521118}, "gps": {"x": 10.065970954624072, "y": 2.1289603073509795}, "landmarks": [{"id": "3", "range": 4.80930668376744, "bearing": -2.972110815448997}, {"id": "4", "range": 3.672778704132146, "bearing": 2.9250186093264308}, {"id": "5", "range": 3.081025638609965, "bearing": 2.4046275857140067}]}, {"step": 50, "t": 25.0, "odom": {"dx": 0.24754920052863325, "dy": -0.038957678418157296, "dtheta": -0.04192465441849101}, "gps": {"x": 10.608647836787453, "y": 2.00923358243315}, "landmarks": [{"id": "3", "range": 5.02905614458022, "bearing": -3.0027453605595045}, {"id": "4", "range": 3.8604552452666763, "bearing": 2.94684710191839}, {"id": "5", "range": 3.6398637698526346, "bearing": 2.4287399299614805}]}, {"step": 51, "t": 25.5, "odom": {"dx": 0.2745816346590286, "dy": -0.012805767161139164, "dtheta": 0.033392646933696334}, "gps": {"x": 10.750740791169342, "y": 1.62484307371623}, "landmarks": [{"id": "4", "range": 4.12659131809949, "bearing": 2.927168917571594}, {"id": "5", "range": 3.8120023027133962, "bearing": 2.4504653128619402}]}, {"step": 52, "t": 26.0, "odom": {"dx": 0.26953460771416193, "dy": -0.016550455343191373, "dtheta": 0.010412469656588888}, "gps": {"x": 10.639474033123218, "y": 1.5070431717656736}, "landmarks": [{"id": "4", "range": 4.298540038871713, "bearing": 2.9946854131691323}, {"id": "5", "range": 3.84028775292027, "bearing": 2.458942227278869}]}, {"step": 53, "t": 26.5, "odom": {"dx": 0.19666946170213073, "dy": 0.007666716623619797, "dtheta": -0.006785628769774892}, "gps": {"x": 11.0694419470034, "y": 1.4073385604751454}, "landmarks": [{"id": "4", "range": 4.480094787030317, "bearing": 2.9061812654786507}, {"id": "5", "range": 4.135559882832695, "bearing": 2.583669982963224}]}, {"step": 54, "t": 27.0, "odom": {"dx": 0.24152159022198816, "dy": 0.011710367538477097, "dtheta": 0.05548826342641534}, "gps": {"x": 11.247930704268386, "y": 1.1517699494615885}, "landmarks": [{"id": "4", "range": 4.897415200692686, "bearing": 2.9692933443242255}, {"id": "5", "range": 4.504524782464131, "bearing": 2.511629024826199}]}, {"step": 55, "t": 27.5, "odom": {"dx": 0.2600842178537674, "dy": 0.021469385572850518, "dtheta": 0.029172913801995205}, "gps": {"x": 11.013036076897068, "y": 0.7645526994532038}, "landmarks": [{"id": "5", "range": 4.656415969233047, "bearing": 2.5484657564513613}]}, {"step": 56, "t": 28.0, "odom": {"dx": 0.260841997977341, "dy": 0.02140509115197769, "dtheta": -0.06299638591540903}, "gps": {"x": 11.353583395999435, "y": 0.9573460635464699}, "landmarks": [{"id": "5", "range": 4.741677816340486, "bearing": 2.57671518082344}]}, {"step": 57, "t": 28.5, "odom": {"dx": 0.24952542899357794, "dy": -0.010325050771330996, "dtheta": -0.030646700245097518}, "gps": {"x": 11.528637900949828, "y": 0.520098272391447}, "landmarks": []}, {"step": 58, "t": 29.0, "odom": {"dx": 0.23936937545663112, "dy": -0.024518039617676443, "dtheta": 0.01912706586210473}, "gps": {"x": 11.714736026513094, "y": 0.26850000431198306}, "landmarks": []}, {"step": 59, "t": 29.5, "odom": {"dx": 0.2446358247597065, "dy": 0.0032828157113811043, "dtheta": 0.02511416208358789}, "gps": {"x": 11.806025655092684, "y": 0.16464216161815443}, "landmarks": []}]
```

## Contract
Implement solution.py with exactly this interface:

def make_estimator(public_spec):
    """Return a NEW estimator object. One object per episode; the verifier creates a fresh one for every
    trial, so no state may persist across episodes (no module-level caches or globals)."""

class Estimator:
    def step(self, new_measurements):
        """new_measurements is a dict, always containing "step" (int) and "t" (seconds):
          step 0:  {"step": 0, "t": 0.0, "odom": None, "gps": {"x","y"} or None, "landmarks": [...]}
          step k:  {"step": k, "t": k*dt, "odom": {"dx","dy","dtheta"} for transition k-1 -> k,
                    "gps": {"x","y"} or None, "landmarks": [{"id","range","bearing"}, ...] (possibly empty)}
        Returns (pose [x, y, theta], covariance 3x3 in local tangent frame of pose)."""

def choose_route(public_spec):
    """Returns {"route": "A" or "B", "success_prob": float, "reasoning": str}."""

Incremental (e.g. iSAM2) or batch inside step() are both allowed. Optional:
build_graph(measurements, public_spec) -> (gtsam.NonlinearFactorGraph, gtsam.Values) for diagnostics.
Same measurements fed to two fresh estimators must give identical outputs (deterministic).
The robot is driven by a pure-pursuit controller that uses YOUR estimator's pose, so choose_route should
reflect the collision-free arrival probability with your own estimator in the loop.
Available in the sandbox: Python 3.12, numpy, scipy, gtsam. No network, no files except what you write.
Answer with ONE ```python fenced block containing the complete solution.py.

## Revision 2 of 2: your previous solution.py
```python
import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import X, L


def _parse_landmarks(spec):
    lm_dict = {}
    for k, v in spec.get("landmarks", {}).items():
        lm_dict[int(k)] = (float(v[0]), float(v[1]))
    return lm_dict


class Estimator:
    def __init__(self, public_spec):
        self.spec = public_spec
        self.landmarks = _parse_landmarks(public_spec)

        # Noise models
        init_vars = public_spec["initial_pose"]["cov_diag_variances"]
        self.init_mean = public_spec["initial_pose"]["mean"]
        self.prior_noise = gtsam.noiseModel.Diagonal.Variances(
            np.array(init_vars, dtype=float)
        )

        sn = public_spec["sensor_noise_std"]
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array(sn["odom"], dtype=float)
        )
        self.br_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([float(sn["bearing"]), float(sn["range"])], dtype=float)
        )
        gps_std = float(sn["gps"])
        self.gps_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([gps_std, gps_std], dtype=float)
        )
        self.landmark_tight_noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([1e-9, 1e-9], dtype=float)
        )

        # Units
        in_units = public_spec.get("input_units", {})
        self.bearing_deg = in_units.get("bearing", "rad") == "deg"

        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()
        self.added_landmarks = set()
        self.last_pose = None
        self.last_step = -1

    def _add_landmark_if_needed(self, lm_id):
        if lm_id not in self.added_landmarks and lm_id in self.landmarks:
            lx, ly = self.landmarks[lm_id]
            pt = gtsam.Point2(lx, ly)
            key = L(lm_id)
            self.values.insert(key, pt)
            self.graph.add(
                gtsam.PriorFactorPoint2(key, pt, self.landmark_tight_noise)
            )
            self.added_landmarks.add(lm_id)

    def step(self, new_measurements):
        k = int(new_measurements["step"])
        key_k = X(k)

        if k == 0:
            init_pose = gtsam.Pose2(
                float(self.init_mean[0]),
                float(self.init_mean[1]),
                float(self.init_mean[2]),
            )
            self.graph.add(
                gtsam.PriorFactorPose2(key_k, init_pose, self.prior_noise)
            )
            if new_measurements.get("gps") is not None:
                gx = float(new_measurements["gps"]["x"])
                gy = float(new_measurements["gps"]["y"])
                curr_pose = gtsam.Pose2(gx, gy, float(self.init_mean[2]))
            else:
                curr_pose = init_pose
            self.values.insert(key_k, curr_pose)
        else:
            odom = new_measurements.get("odom")
            if odom is not None:
                dx = float(odom["dx"])
                dy = float(odom["dy"])
                dtheta = float(odom["dtheta"])
                if self.bearing_deg:
                    # Just in case, though odom angle is always rad unless specified
                    pass
                odom_pose = gtsam.Pose2(dx, dy, dtheta)
            else:
                odom_pose = gtsam.Pose2(0.0, 0.0, 0.0)

            prev_key = X(self.last_step)
            self.graph.add(
                gtsam.BetweenFactorPose2(
                    prev_key, key_k, odom_pose, self.odom_noise
                )
            )
            curr_pose = self.last_pose.compose(odom_pose)
            self.values.insert(key_k, curr_pose)

        # GPS measurement
        gps = new_measurements.get("gps")
        if gps is not None:
            gx = float(gps["x"])
            gy = float(gps["y"])
            self.graph.add(
                gtsam.GPSFactor2(key_k, gtsam.Point2(gx, gy), self.gps_noise)
            )

        # Landmark measurements
        lms = new_measurements.get("landmarks")
        if lms:
            for lm in lms:
                lm_id = int(lm["id"])
                if lm_id not in self.landmarks:
                    continue
                self._add_landmark_if_needed(lm_id)
                r = float(lm["range"])
                b = float(lm["bearing"])
                if self.bearing_deg:
                    b = math.radians(b)
                self.graph.add(
                    gtsam.BearingRangeFactor2D(
                        key_k, L(lm_id), gtsam.Rot2(b), r, self.br_noise
                    )
                )

        # Optimize
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph, self.values, params
        )
        self.values = optimizer.optimize()

        est_pose = self.values.atPose2(key_k)
        self.last_pose = est_pose
        self.last_step = k

        marginals = gtsam.Marginals(self.graph, self.values)
        cov = marginals.marginalCovariance(key_k)

        pose_out = [
            float(est_pose.x()),
            float(est_pose.y()),
            float(est_pose.theta()),
        ]
        return pose_out, np.array(cov, dtype=float)


def make_estimator(public_spec):
    return Estimator(public_spec)


def build_graph(measurements, public_spec):
    est = make_estimator(public_spec)
    for m in measurements:
        est.step(m)
    return est.graph, est.values


def _point_to_segment_dist(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx * dx + dy * dy
    if l2 == 0.0:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def _pure_pursuit_control(pose, waypoints, lookahead, v_cmd, omega_max, wp_idx):
    x, y, theta = pose
    n_wp = len(waypoints)

    # Advance waypoint index if close
    while wp_idx < n_wp - 1:
        wx, wy = waypoints[wp_idx + 1]
        if math.hypot(wx - x, wy - y) < lookahead:
            wp_idx += 1
        else:
            break

    # Find lookahead point on path segments starting from wp_idx
    target_x, target_y = waypoints[-1]
    found = False
    for i in range(wp_idx, n_wp - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        fx = x1 - x
        fy = y1 - y
        a = dx * dx + dy * dy
        if a < 1e-12:
            continue
        b = 2.0 * (fx * dx + fy * dy)
        c = (fx * fx + fy * fy) - lookahead * lookahead
        disc = b * b - 4.0 * a * c
        if disc >= 0.0:
            sqrt_disc = math.sqrt(disc)
            t1 = (-b - sqrt_disc) / (2.0 * a)
            t2 = (-b + sqrt_disc) / (2.0 * a)
            # Prefer further along segment
            if 0.0 <= t2 <= 1.0:
                target_x = x1 + t2 * dx
                target_y = y1 + t2 * dy
                found = True
                break
            elif 0.0 <= t1 <= 1.0:
                target_x = x1 + t1 * dx
                target_y = y1 + t1 * dy
                found = True
                break

    if not found:
        # Project onto current segment or pick next waypoint
        if wp_idx < n_wp - 1:
            x1, y1 = waypoints[wp_idx]
            x2, y2 = waypoints[wp_idx + 1]
            dx, dy = x2 - x1, y2 - y1
            l2 = dx * dx + dy * dy
            if l2 > 1e-12:
                t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / l2))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
                rem = math.hypot(x2 - proj_x, y2 - proj_y)
                if rem >= lookahead:
                    seg_len = math.sqrt(l2)
                    target_x = proj_x + (dx / seg_len) * lookahead
                    target_y = proj_y + (dy / seg_len) * lookahead
                else:
                    target_x, target_y = x2, y2
            else:
                target_x, target_y = x2, y2
        else:
            target_x, target_y = waypoints[-1]

    # Transform target to robot frame
    dx_w = target_x - x
    dy_w = target_y - y
    local_x = math.cos(theta) * dx_w + math.sin(theta) * dy_w
    local_y = -math.sin(theta) * dx_w + math.cos(theta) * dy_w

    l_sq = local_x * local_x + local_y * local_y
    if l_sq < 1e-9:
        omega = 0.0
    else:
        curvature = 2.0 * local_y / l_sq
        omega = v_cmd * curvature

    omega = max(-omega_max, min(omega_max, omega))
    return v_cmd, omega, wp_idx


def _simulate_route(route_waypoints, spec, rng):
    dt = float(spec["dt"])
    max_steps = int(spec["max_steps"])
    arrival_tol = float(spec["arrival_tolerance"])
    robot_radius = float(spec["robot_radius"])
    walls = spec.get("walls", [])
    landmarks = _parse_landmarks(spec)
    max_range = float(spec["landmark_visibility"]["max_range"])

    init_mean = spec["initial_pose"]["mean"]
    init_vars = spec["initial_pose"]["cov_diag_variances"]
    true_x = float(init_mean[0]) + rng.normal(0.0, math.sqrt(init_vars[0]))
    true_y = float(init_mean[1]) + rng.sqrt(init_vars[1]) if False else float(init_mean[1]) + rng.normal(0.0, math.sqrt(init_vars[1]))
    true_theta = float(init_mean[2]) + rng.normal(0.0, math.sqrt(init_vars[2]))

    sn = spec["sensor_noise_std"]
    odom_std = sn["odom"]
    range_std = float(sn["range"])
    bearing_std = float(sn["bearing"])
    gps_std = float(sn["gps"])

    an = spec["actuator_noise_std"]
    v_std = float(an["v"])
    omega_std = float(an["omega"])

    gps_sched = spec["gps_schedule"]
    gps_rate = int(gps_sched["rate_steps"])
    dropout = gps_sched.get("dropout_steps", [-1, -1])
    drop_start, drop_end = int(dropout[0]), int(dropout[1])

    ctrl = spec["controller"]
    lookahead = float(ctrl["lookahead"])
    v_cmd = float(ctrl["v"])
    omega_max = float(ctrl["omega_max"])

    est = make_estimator(spec)
    goal_x, goal_y = route_waypoints[-1]
    wp_idx = 0

    prev_true_pose = gtsam.Pose2(true_x, true_y, true_theta)

    for k in range(max_steps):
        t = k * dt
        curr_true_pose = gtsam.Pose2(true_x, true_y, true_theta)

        # Check collision
        for w in walls:
            dist = _point_to_segment_dist(true_x, true_y, w[0], w[1], w[2], w[3])
            if dist <= robot_radius:
                return False

        # Check arrival
        if math.hypot(true_x - goal_x, true_y - goal_y) <= arrival_tol:
            return True

        # Generate measurements
        if k == 0:
            odom_meas = None
        else:
            rel = prev_true_pose.between(curr_true_pose)
            # Tangent noise on relative pose
            xi = gtsam.Pose2.Logmap(rel)
            noisy_xi = np.array([
                xi[0] + rng.normal(0.0, odom_std[0]),
                xi[1] + rng.normal(0.0, odom_std[1]),
                xi[2] + rng.normal(0.0, odom_std[2]),
            ])
            noisy_rel = gtsam.Pose2.Expmap(noisy_xi)
            odom_meas = {
                "dx": float(noisy_rel.x()),
                "dy": float(noisy_rel.y()),
                "dtheta": float(noisy_rel.theta()),
            }

        # GPS
        if (k % gps_rate == 0) and not (drop_start <= k <= drop_end):
            gps_meas = {
                "x": true_x + rng.normal(0.0, gps_std),
                "y": true_y + rng.normal(0.0, gps_std),
            }
        else:
            gps_meas = None

        # Landmarks
        lm_meas = []
        for lm_id, (lx, ly) in sorted(landmarks.items()):
            r_true = math.hypot(lx - true_x, ly - true_y)
            if r_true <= max_range:
                b_true = math.atan2(ly - true_y, lx - true_x) - true_theta
                b_true = math.atan2(math.sin(b_true), math.cos(b_true))
                r_m = r_true + rng.normal(0.0, range_std)
                b_m = b_true + rng.normal(0.0, bearing_std)
                b_m = math.atan2(math.sin(b_m), math.cos(b_m))
                lm_meas.append({
                    "id": str(lm_id),
                    "range": r_m,
                    "bearing": b_m,
                })

        meas = {
            "step": k,
            "t": t,
            "odom": odom_meas,
            "gps": gps_meas,
            "landmarks": lm_meas,
        }

        est_pose, _ = est.step(meas)

        # Control based on estimated pose
        v_c, omega_c, wp_idx = _pure_pursuit_control(
            est_pose, route_waypoints, lookahead, v_cmd, omega_max, wp_idx
        )

        # Propagate true state with actuator noise
        v_actual = v_c + rng.normal(0.0, v_std)
        omega_actual = omega_c + rng.normal(0.0, omega_std)

        prev_true_pose = curr_true_pose
        # Exact arc or Euler integration
        if abs(omega_actual) < 1e-6:
            true_x += v_actual * dt * math.cos(true_theta)
            true_y += v_actual * dt * math.sin(true_theta)
            true_theta += omega_actual * dt
        else:
            # Body-frame velocity integration via Pose2 Expmap or standard unicycle
            dx_b = v_actual * dt
            dtheta_b = omega_actual * dt
            step_pose = gtsam.Pose2.Expmap(np.array([dx_b, 0.0, dtheta_b]))
            next_pose = curr_true_pose.compose(step_pose)
            true_x = float(next_pose.x())
            true_y = float(next_pose.y())
            true_theta = float(next_pose.theta())

    return False


def choose_route(public_spec):
    routes = public_spec.get("routes", {})
    rng = np.random.default_rng(42)
    n_trials = 25

    results = {}
    lengths = {}
    for r_name, wps in routes.items():
        pts = [(float(p[0]), float(p[1])) for p in wps]
        dist = sum(
            math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)
        )
        lengths[r_name] = dist
        successes = 0
        for _ in range(n_trials):
            if _simulate_route(pts, public_spec, rng):
                successes += 1
        results[r_name] = successes / float(n_trials)

    # Choose route maximizing success_prob; distance breaks close ties (within 0.05)
    best_route = None
    best_prob = -1.0
    for r_name in sorted(routes.keys()):
        prob = results[r_name]
        if best_route is None:
            best_route = r_name
            best_prob = prob
        else:
            if prob > best_prob + 0.05:
                best_route = r_name
                best_prob = prob
            elif abs(prob - best_prob) <= 0.05:
                if lengths[r_name] < lengths[best_route]:
                    best_route = r_name
                    best_prob = prob

    reasoning = (
        f"Route B avoids the narrow wall corridor at y=[-0.45, 0.45], x=[5.4, 9.0] "
        f"where GPS drops out (steps 20-35) and landmarks at y=5.5 are out of range "
        f"(>5.0m), causing high lateral uncertainty and wall collisions on Route A. "
        f"Route B passes at y=3.0 within range of landmarks 0..5 during GPS dropout, "
        f"maintaining tight localization and achieving {results.get('B', 1.0):.2f} "
        f"collision-free arrival vs {results.get('A', 0.0):.2f} for Route A."
    )

    return {
        "route": best_route,
        "success_prob": float(best_prob),
        "reasoning": reasoning,
    }

```

## Verifier feedback on that solution
- Execution failed: AgentError: TypeError: __init__(): incompatible constructor arguments. The following argument types are supported:
    1. gtsam.gtsam.GPSFactor2(key: typing.SupportsInt | typing.SupportsIndex, gpsIn: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], model: gtsam.gtsam.noiseModel.Base)

Invoked with: 8646911284551352320, array([-0.06854817,  0.27129691]), isotropic dim=2 sigma=0.15

Traceback (most recent call last):
  File "agent_worker.py", line 70, in main
    pose, cov = est.step(req["m"])
                ^^^^^^^^^^^^^^^^^^
  File "solution.py", line 110, in step
    gtsam.GPSFactor2(key_k, gtsam.Point2(gx, gy), self.gps_noise)
TypeError: __init__(): incompatible constructor arguments. The following argument types are supported:
    1. gtsam.gtsam.GPSFactor2(key: typing.SupportsInt | typing.SupportsIndex, gpsIn: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[3, 1]"], model: gtsam.gtsam.noiseModel.Base)

Invoked with: 8646911284551352320, array([-0.06854817,  0.27129691]), isotropic dim=2 sigma=0.15



Fix the failed checks. Return the complete revised solution.py.