# SimCheck task

Write `solution.py` implementing `make_estimator(public_spec)`, `Estimator.step(new_measurements)` and
`choose_route(public_spec)` as described in the contract. Use GTSAM. Read `spec.json` for every parameter.

Conventions:
- The true initial pose x_0 is sampled from the stated prior N(initial_pose.mean, diag(cov_diag_variances));
  it is not known exactly.
- `initial_pose.cov_diag_variances` are VARIANCES. Every `*_noise_std` entry is a STANDARD DEVIATION and is
  always in SI units (metres, radians), even when measurements arrive in mm or degrees (`input_units`).
- Odometry noise is Pose2 tangent-space noise on the relative pose (x_{k-1} -> x_k), body frame.
- Landmark bearing is measured relative to the robot heading. GPS gives world-frame x, y.
- Returned covariance is 3x3 in the local tangent frame of the returned pose (the GTSAM Pose2 marginal
  convention, ordered x, y, theta), not in the world frame.

Grading: trajectory accuracy, consistency of the reported uncertainty across independent runs, and
collision-free arrival rate of the chosen route in closed-loop simulation driven by your estimator.

Scenario-specific notes:
- ODOMETRY NOISE: odometry noise std is between 0.5x and 3x the nominal value in `sensor_noise_std.odom`; the exact value is not given (`odom_noise_scale_range`). Other noise values are exact.


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
  "objective": "maximize collision-free arrival probability; distance breaks close ties",
  "odom_noise_scale_range": [
    0.5,
    3.0
  ]
}
```

## measurements.json (one example episode; your estimator is graded on many others)
```json
[{"step": 0, "t": 0.0, "odom": null, "gps": {"x": 0.13663820571183286, "y": 0.1343708992580851}, "landmarks": []}, {"step": 1, "t": 0.5, "odom": {"dx": 0.2022416432863924, "dy": 0.05379822140479541, "dtheta": 0.4878203075370722}, "gps": {"x": 0.18842822473705073, "y": 0.31429933136440874}, "landmarks": []}, {"step": 2, "t": 1.0, "odom": {"dx": 0.23189941082289922, "dy": 0.053718551623195, "dtheta": 0.37891813972651045}, "gps": {"x": 0.42450313153339486, "y": 0.34348407004872905}, "landmarks": []}, {"step": 3, "t": 1.5, "odom": {"dx": 0.23168819483350425, "dy": 0.02575580908822958, "dtheta": 0.20595539788054903}, "gps": {"x": 0.6478782800937313, "y": 0.46327206519939385}, "landmarks": []}, {"step": 4, "t": 2.0, "odom": {"dx": 0.19384324844346065, "dy": 0.015738583576998977, "dtheta": 0.06357164152480178}, "gps": {"x": 0.8539767556046849, "y": 0.5813639999964195}, "landmarks": [{"id": "0", "range": 5.110360791131293, "bearing": 0.34209649389938734}]}, {"step": 5, "t": 2.5, "odom": {"dx": 0.2242914002894483, "dy": -0.0021062920205904842, "dtheta": -0.04792701967080908}, "gps": {"x": 0.8017303453682859, "y": 0.9252003537596662}, "landmarks": [{"id": "0", "range": 4.9047422872793485, "bearing": 0.40176960010492824}]}, {"step": 6, "t": 3.0, "odom": {"dx": 0.2571990665596204, "dy": 0.013298292595894146, "dtheta": -0.02455922724487862}, "gps": {"x": 0.8319741316746647, "y": 0.9285662469924532}, "landmarks": [{"id": "0", "range": 4.577895257525972, "bearing": 0.5068719521672924}]}, {"step": 7, "t": 3.5, "odom": {"dx": 0.28296162171079925, "dy": -0.01581876388351111, "dtheta": -0.09570723162638692}, "gps": {"x": 1.2439247447543322, "y": 1.1558765796115316}, "landmarks": [{"id": "0", "range": 4.381105622556441, "bearing": 0.595504982368527}, {"id": "1", "range": 4.953023841956641, "bearing": 0.14897369840430708}]}, {"step": 8, "t": 4.0, "odom": {"dx": 0.24009600174850165, "dy": -0.011318797296236183, "dtheta": 0.008551650713813525}, "gps": {"x": 1.2712952301276197, "y": 1.5147646713936274}, "landmarks": [{"id": "0", "range": 3.9860117964248154, "bearing": 0.6233375241586474}, {"id": "1", "range": 4.342048118919046, "bearing": 0.22003084633107806}]}, {"step": 9, "t": 4.5, "odom": {"dx": 0.24237331903985743, "dy": -0.0043573071041231835, "dtheta": 0.013355773080845382}, "gps": {"x": 1.5200903829036054, "y": 1.631098645699427}, "landmarks": [{"id": "0", "range": 3.8442782983974713, "bearing": 0.6561909748946464}, {"id": "1", "range": 4.2635580416917565, "bearing": 0.22565500081969248}]}, {"step": 10, "t": 5.0, "odom": {"dx": 0.21731767783616462, "dy": -0.001745213608456835, "dtheta": -0.05732134225308294}, "gps": {"x": 1.287475676147435, "y": 1.58873828797331}, "landmarks": [{"id": "0", "range": 3.7937869142652096, "bearing": 0.6491856220999344}, {"id": "1", "range": 3.9359625105698504, "bearing": 0.30182431337813087}]}, {"step": 11, "t": 5.5, "odom": {"dx": 0.24658240545789678, "dy": -0.00835781627565921, "dtheta": 0.015698369131350868}, "gps": {"x": 1.8574160402927269, "y": 1.8655241196220853}, "landmarks": [{"id": "0", "range": 3.7071354463145023, "bearing": 0.7394520933283504}, {"id": "1", "range": 3.797896694715755, "bearing": 0.20037345441275534}, {"id": "2", "range": 4.954086627710717, "bearing": -0.16786642477152558}]}, {"step": 12, "t": 6.0, "odom": {"dx": 0.20886633497335033, "dy": 0.014151335336594855, "dtheta": 0.0036471663078108}, "gps": {"x": 1.3855240432742, "y": 2.1682395454899512}, "landmarks": [{"id": "0", "range": 3.3180228705621704, "bearing": 0.7842378802829035}, {"id": "1", "range": 3.4320434781898466, "bearing": 0.2630183591323094}, {"id": "2", "range": 4.828626878419957, "bearing": -0.14609693357658093}]}, {"step": 13, "t": 6.5, "odom": {"dx": 0.2775406012914257, "dy": -0.009717438865868018, "dtheta": -0.01710842280891996}, "gps": {"x": 1.8876449824162365, "y": 2.5141537221092434}, "landmarks": [{"id": "0", "range": 3.3148514173190073, "bearing": 0.8321736719507395}, {"id": "1", "range": 3.3812018270123216, "bearing": 0.3121003982656547}, {"id": "2", "range": 4.500672530555434, "bearing": -0.14101336846673052}]}, {"step": 14, "t": 7.0, "odom": {"dx": 0.2684026046901729, "dy": 0.01587635850747254, "dtheta": -0.006055349666604666}, "gps": {"x": 1.7971128244343983, "y": 2.604654583970136}, "landmarks": [{"id": "0", "range": 3.2218850122479012, "bearing": 1.031543091981355}, {"id": "1", "range": 3.088084115524033, "bearing": 0.3425721621180351}, {"id": "2", "range": 4.13719937811221, "bearing": -0.07154400604028321}]}, {"step": 15, "t": 7.5, "odom": {"dx": 0.27020705430433656, "dy": -0.02096389301006659, "dtheta": -0.1432718513131447}, "gps": {"x": 1.9696857480763401, "y": 3.013733489414414}, "landmarks": [{"id": "0", "range": 2.944255716086462, "bearing": 1.1813261477025128}, {"id": "1", "range": 2.8373579470129404, "bearing": 0.5011521670286223}, {"id": "2", "range": 4.015347039552876, "bearing": 0.015973882864817046}]}, {"step": 16, "t": 8.0, "odom": {"dx": 0.2865875730546437, "dy": -0.05159913203048468, "dtheta": -0.28265771225459135}, "gps": {"x": 2.317155673977116, "y": 2.8365419696077567}, "landmarks": [{"id": "0", "range": 2.8536367566825263, "bearing": 1.5521340002577375}, {"id": "1", "range": 2.7963536183670437, "bearing": 0.9213917968853806}, {"id": "2", "range": 3.634489704105418, "bearing": 0.3275802989497252}]}, {"step": 17, "t": 8.5, "odom": {"dx": 0.2234685435054411, "dy": -0.016734041758342984, "dtheta": -0.20956693606346835}, "gps": {"x": 2.626533667347419, "y": 3.0015923970162732}, "landmarks": [{"id": "0", "range": 2.805282482556332, "bearing": 1.795416637098631}, {"id": "1", "range": 2.5934319354676143, "bearing": 1.1278413275496586}, {"id": "2", "range": 3.5992011111613036, "bearing": 0.5317490848606088}, {"id": "3", "range": 4.872875979700818, "bearing": 0.2766464652781875}]}, {"step": 18, "t": 9.0, "odom": {"dx": 0.276606460056247, "dy": -0.032910739905747666, "dtheta": -0.1875862654246444}, "gps": {"x": 3.3393525092720404, "y": 2.76233562817205}, "landmarks": [{"id": "0", "range": 3.0335967984891554, "bearing": 2.1371934071066736}, {"id": "1", "range": 2.4348043872413134, "bearing": 1.4241911516271788}, {"id": "2", "range": 3.1786498834241925, "bearing": 0.7572251877062941}, {"id": "3", "range": 4.90793767348058, "bearing": 0.5159335545066615}]}, {"step": 19, "t": 9.5, "odom": {"dx": 0.26432016762632715, "dy": -0.012975176792079931, "dtheta": -0.10232315827981187}, "gps": {"x": 3.2751366089272778, "y": 3.3424169327142663}, "landmarks": [{"id": "0", "range": 3.1682900618968413, "bearing": 2.255613796101099}, {"id": "1", "range": 2.5129543750607533, "bearing": 1.619507588145904}, {"id": "2", "range": 2.9477412174702584, "bearing": 0.9428562795519024}, {"id": "3", "range": 4.444315284058338, "bearing": 0.5102239523977858}]}, {"step": 20, "t": 10.0, "odom": {"dx": 0.24232134075276834, "dy": -0.00811771286929845, "dtheta": 0.006142929022851293}, "gps": null, "landmarks": [{"id": "0", "range": 3.403699827723926, "bearing": 2.2614851120322452}, {"id": "1", "range": 2.4097365098842394, "bearing": 1.671156396593961}, {"id": "2", "range": 2.9670454697335864, "bearing": 0.972448831370115}, {"id": "3", "range": 4.199744791171284, "bearing": 0.6129918577643534}]}, {"step": 21, "t": 10.5, "odom": {"dx": 0.2610228271796507, "dy": -0.0021724252112369125, "dtheta": -0.03582462600777573}, "gps": null, "landmarks": [{"id": "0", "range": 3.4044655508389154, "bearing": 2.358655581092113}, {"id": "1", "range": 2.373469432769761, "bearing": 1.8519458226435006}, {"id": "2", "range": 2.8947358383047153, "bearing": 1.1278492364314765}, {"id": "3", "range": 3.99696532637932, "bearing": 0.6763317761343957}]}, {"step": 22, "t": 11.0, "odom": {"dx": 0.20458632824630454, "dy": 9.272811660048354e-05, "dtheta": -0.010034855993264851}, "gps": null, "landmarks": [{"id": "0", "range": 3.724851435246815, "bearing": 2.4969803067586973}, {"id": "1", "range": 2.4614116859515756, "bearing": 1.946270161564585}, {"id": "2", "range": 2.590912773874494, "bearing": 1.162772404498038}, {"id": "3", "range": 3.8940834893070897, "bearing": 0.7460020572760757}]}, {"step": 23, "t": 11.5, "odom": {"dx": 0.2773466528713165, "dy": -0.012441727815854751, "dtheta": 0.03399222163413295}, "gps": null, "landmarks": [{"id": "0", "range": 4.076357101910881, "bearing": 2.4104420267764652}, {"id": "1", "range": 2.653646923794362, "bearing": 2.0107709788434036}, {"id": "2", "range": 2.6296514374656237, "bearing": 1.2693210577982619}, {"id": "3", "range": 3.6355871222590577, "bearing": 0.7495961226030445}]}, {"step": 24, "t": 12.0, "odom": {"dx": 0.296464141118196, "dy": -0.016696139026120507, "dtheta": 0.011502463522709358}, "gps": null, "landmarks": [{"id": "0", "range": 4.012201951631167, "bearing": 2.4831623547938433}, {"id": "1", "range": 2.7570632775678816, "bearing": 2.1408966921912915}, {"id": "2", "range": 2.331029848625965, "bearing": 1.3823192995745988}, {"id": "3", "range": 3.362826143585098, "bearing": 0.6984926127353934}]}, {"step": 25, "t": 12.5, "odom": {"dx": 0.23685621761233186, "dy": 0.0019198530927817571, "dtheta": -0.031793489401185746}, "gps": null, "landmarks": [{"id": "0", "range": 4.28978238976477, "bearing": 2.5342622771789447}, {"id": "1", "range": 2.8354161682965735, "bearing": 2.159393191954381}, {"id": "2", "range": 2.451880037431971, "bearing": 1.4952305178858705}, {"id": "3", "range": 3.3129358843751557, "bearing": 0.9027841491192161}, {"id": "4", "range": 4.588363824999136, "bearing": 0.5576119326523261}]}, {"step": 26, "t": 13.0, "odom": {"dx": 0.24857785026305587, "dy": 0.0050437859464544516, "dtheta": -0.015812391687160726}, "gps": null, "landmarks": [{"id": "0", "range": 4.438507413363202, "bearing": 2.529819509537562}, {"id": "1", "range": 3.2184623484507418, "bearing": 2.2445522858438522}, {"id": "2", "range": 2.42354902758662, "bearing": 1.5832111320859124}, {"id": "3", "range": 3.2680638057321816, "bearing": 0.9695717885737523}, {"id": "4", "range": 4.685387382354941, "bearing": 0.5696273912399645}]}, {"step": 27, "t": 13.5, "odom": {"dx": 0.2770873136768947, "dy": 0.019018851637228842, "dtheta": 0.044215276078478}, "gps": null, "landmarks": [{"id": "0", "range": 4.660097273376866, "bearing": 2.604350477023731}, {"id": "1", "range": 3.2782280753107917, "bearing": 2.289443110490523}, {"id": "2", "range": 2.708900918022643, "bearing": 1.6038085772217014}, {"id": "3", "range": 3.1021850040473993, "bearing": 0.9077560282239752}, {"id": "4", "range": 4.434275928740035, "bearing": 0.6076014968929542}]}, {"step": 28, "t": 14.0, "odom": {"dx": 0.2903833275940325, "dy": -0.011399344812454547, "dtheta": -0.049151890637200456}, "gps": null, "landmarks": [{"id": "1", "range": 3.492012442178008, "bearing": 2.37693856333847}, {"id": "2", "range": 2.5984860426816314, "bearing": 1.8163490623244645}, {"id": "3", "range": 2.8680587447996757, "bearing": 1.1522483133507713}, {"id": "4", "range": 4.1954525436592665, "bearing": 0.7463468617462774}]}, {"step": 29, "t": 14.5, "odom": {"dx": 0.22930351780027483, "dy": 0.0010659874953580966, "dtheta": 0.02747048945492947}, "gps": null, "landmarks": [{"id": "1", "range": 3.586544064162999, "bearing": 2.414634962408808}, {"id": "2", "range": 2.6087307316176007, "bearing": 1.9364426395934027}, {"id": "3", "range": 2.772772649033576, "bearing": 1.217518101375684}, {"id": "4", "range": 3.935203845414711, "bearing": 0.6706556306215194}]}, {"step": 30, "t": 15.0, "odom": {"dx": 0.22825098647909278, "dy": 0.00862486385227188, "dtheta": 0.03863199065124157}, "gps": null, "landmarks": [{"id": "1", "range": 4.02975057891743, "bearing": 2.400139318694368}, {"id": "2", "range": 2.577469782921034, "bearing": 1.899490600199968}, {"id": "3", "range": 2.745366569737069, "bearing": 1.2034757704583416}, {"id": "4", "range": 3.8727045644553475, "bearing": 0.7146635900658262}]}, {"step": 31, "t": 15.5, "odom": {"dx": 0.2513937698722252, "dy": -0.0011523633375518445, "dtheta": 0.008224085163692421}, "gps": null, "landmarks": [{"id": "1", "range": 4.001254734652165, "bearing": 2.458190296580206}, {"id": "2", "range": 2.9070640876724942, "bearing": 1.9858798640541249}, {"id": "3", "range": 2.6297246769633764, "bearing": 1.177663878550943}, {"id": "4", "range": 3.533831266056678, "bearing": 0.7225691232882872}]}, {"step": 32, "t": 16.0, "odom": {"dx": 0.2318353837527678, "dy": -0.009244313139593018, "dtheta": -0.03943737354791932}, "gps": null, "landmarks": [{"id": "1", "range": 4.252351797642064, "bearing": 2.458999565558379}, {"id": "2", "range": 2.8168545177772297, "bearing": 2.0921159293851446}, {"id": "3", "range": 2.3583120361641337, "bearing": 1.4185184089723917}, {"id": "4", "range": 3.454011706281072, "bearing": 0.824432720103287}, {"id": "5", "range": 5.004344176876963, "bearing": 0.49456061391951867}]}, {"step": 33, "t": 16.5, "odom": {"dx": 0.21131555791704532, "dy": -0.03963627315783334, "dtheta": -0.047326041364878856}, "gps": null, "landmarks": [{"id": "1", "range": 4.411319544221436, "bearing": 2.5529105875431615}, {"id": "2", "range": 2.8260258048158415, "bearing": 2.2415987026934374}, {"id": "3", "range": 2.5526747678518262, "bearing": 1.5270005318681656}, {"id": "4", "range": 3.1115155222080957, "bearing": 0.8807789692211898}, {"id": "5", "range": 4.6532748731179, "bearing": 0.5500853219039139}]}, {"step": 34, "t": 17.0, "odom": {"dx": 0.23882715210218292, "dy": 0.006812495413147675, "dtheta": 0.0190198702247427}, "gps": null, "landmarks": [{"id": "1", "range": 4.702708326551782, "bearing": 2.626946520334153}, {"id": "2", "range": 3.197515822481404, "bearing": 2.277577652412363}, {"id": "3", "range": 2.3669698098503194, "bearing": 1.5498881771879462}, {"id": "4", "range": 3.1750461448047846, "bearing": 0.923833633318309}, {"id": "5", "range": 4.511861372523382, "bearing": 0.6046127942218265}]}, {"step": 35, "t": 17.5, "odom": {"dx": 0.24887427852390448, "dy": -0.01600098046633545, "dtheta": 0.029210491097962105}, "gps": null, "landmarks": [{"id": "1", "range": 4.918995648867486, "bearing": 2.5409125475587313}, {"id": "2", "range": 3.2579670976767545, "bearing": 2.245823386101492}, {"id": "3", "range": 2.510884479687322, "bearing": 1.6479565739681004}, {"id": "4", "range": 2.9802276651980617, "bearing": 0.9926668656401887}, {"id": "5", "range": 4.4688081892424645, "bearing": 0.5686461094734176}]}, {"step": 36, "t": 18.0, "odom": {"dx": 0.31697758031604767, "dy": 0.007396629919516989, "dtheta": -0.04867677213180134}, "gps": {"x": 7.418973779445276, "y": 2.876404374873817}, "landmarks": [{"id": "2", "range": 3.6920010518193993, "bearing": 2.38917496432113}, {"id": "3", "range": 2.5155449870228024, "bearing": 1.7701498335941652}, {"id": "4", "range": 2.870538132882807, "bearing": 1.0409505656464662}, {"id": "5", "range": 4.165342134402678, "bearing": 0.6531792765577253}]}, {"step": 37, "t": 18.5, "odom": {"dx": 0.24955909381601316, "dy": -0.016750116507735075, "dtheta": 0.030452513427138568}, "gps": {"x": 7.836926418834074, "y": 2.960019588018122}, "landmarks": [{"id": "2", "range": 3.8078279798218895, "bearing": 2.367794731923297}, {"id": "3", "range": 2.385315707122018, "bearing": 1.8487190068193105}, {"id": "4", "range": 2.627675212663021, "bearing": 1.1457363652444688}, {"id": "5", "range": 3.9827923985109774, "bearing": 0.695918451516393}]}, {"step": 38, "t": 19.0, "odom": {"dx": 0.28340801667835486, "dy": -0.008378575924061835, "dtheta": 0.01245096319577915}, "gps": {"x": 8.076287197316518, "y": 2.9064006336435315}, "landmarks": [{"id": "2", "range": 3.8615895722302365, "bearing": 2.440216579643196}, {"id": "3", "range": 2.584196895396904, "bearing": 1.9592462171199907}, {"id": "4", "range": 2.6811334976336902, "bearing": 1.225149994608726}, {"id": "5", "range": 3.7604414094101712, "bearing": 0.6929797315316009}]}, {"step": 39, "t": 19.5, "odom": {"dx": 0.24199008420393522, "dy": 0.021509735368375738, "dtheta": 0.002173181084159795}, "gps": {"x": 8.368867040143815, "y": 2.96902444426578}, "landmarks": [{"id": "2", "range": 4.077475990057113, "bearing": 2.455993860681124}, {"id": "3", "range": 2.8382203555294474, "bearing": 2.010344284297731}, {"id": "4", "range": 2.4099743499735564, "bearing": 1.3086221739575414}, {"id": "5", "range": 3.50129892894011, "bearing": 0.7498399117111001}]}, {"step": 40, "t": 20.0, "odom": {"dx": 0.22796952442190693, "dy": -0.002251185150184783, "dtheta": 0.026327745134444844}, "gps": {"x": 8.415940990502019, "y": 2.947993722555002}, "landmarks": [{"id": "2", "range": 4.241046035829062, "bearing": 2.4331838485022317}, {"id": "3", "range": 2.8745619717980704, "bearing": 2.0587302820034843}, {"id": "4", "range": 2.428919874699211, "bearing": 1.3755363712296838}, {"id": "5", "range": 3.3789750592736967, "bearing": 0.7408030113337079}]}, {"step": 41, "t": 20.5, "odom": {"dx": 0.2620894005669538, "dy": -0.010814125859826284, "dtheta": -0.03933830582047886}, "gps": {"x": 8.769090554239739, "y": 3.090144857489436}, "landmarks": [{"id": "2", "range": 4.5083150158770495, "bearing": 2.5651823922118844}, {"id": "3", "range": 3.0363185923381106, "bearing": 2.1733895089045454}, {"id": "4", "range": 2.4681423262934183, "bearing": 1.5118659863889103}, {"id": "5", "range": 3.1599870683159828, "bearing": 0.8533798333623364}]}, {"step": 42, "t": 21.0, "odom": {"dx": 0.2600497992542425, "dy": 0.008627095750451699, "dtheta": -0.01134704449614718}, "gps": {"x": 8.889036353045322, "y": 2.9942007626063623}, "landmarks": [{"id": "2", "range": 4.713765793617924, "bearing": 2.5744331884432947}, {"id": "3", "range": 3.2611733651702934, "bearing": 2.225297040277688}, {"id": "4", "range": 2.6101625810305644, "bearing": 1.6194543320054686}, {"id": "5", "range": 3.085842518738431, "bearing": 0.944239673253982}]}, {"step": 43, "t": 21.5, "odom": {"dx": 0.2291291725859624, "dy": -0.0030356388010733364, "dtheta": -0.16024664671790548}, "gps": {"x": 9.133500501068445, "y": 3.061638640140019}, "landmarks": [{"id": "2", "range": 4.74436439466199, "bearing": 2.7250375979590498}, {"id": "3", "range": 3.2273724956347185, "bearing": 2.507714233725922}, {"id": "4", "range": 2.5114017507080133, "bearing": 1.7965931055436126}, {"id": "5", "range": 3.0373397173619896, "bearing": 1.1476502785105573}]}, {"step": 44, "t": 22.0, "odom": {"dx": 0.26028156038538847, "dy": -0.03082074631277714, "dtheta": -0.16362395717181483}, "gps": {"x": 9.63421480719102, "y": 2.93831241272252}, "landmarks": [{"id": "3", "range": 3.365738446619322, "bearing": 2.6902148549691276}, {"id": "4", "range": 2.7902456312433643, "bearing": 2.1325744508462723}, {"id": "5", "range": 2.8041490237580415, "bearing": 1.3794307347437824}]}, {"step": 45, "t": 22.5, "odom": {"dx": 0.22445518507745166, "dy": -0.05133447388995145, "dtheta": -0.2521472198135466}, "gps": {"x": 9.542238318801305, "y": 2.982440592138275}, "landmarks": [{"id": "3", "range": 3.811793842044309, "bearing": 2.9434662967506835}, {"id": "4", "range": 2.8358544637526313, "bearing": 2.4367469034785696}, {"id": "5", "range": 3.121395550188959, "bearing": 1.7254414708041423}]}, {"step": 46, "t": 23.0, "odom": {"dx": 0.2765894922911604, "dy": -0.012027973944773378, "dtheta": -0.09555654085225544}, "gps": {"x": 9.871892601691192, "y": 2.812398123734166}, "landmarks": [{"id": "3", "range": 3.9750233042650485, "bearing": 3.0583568107099053}, {"id": "4", "range": 2.947047128358335, "bearing": 2.6002008845914224}, {"id": "5", "range": 2.9948213422662477, "bearing": 1.8942974848088916}]}, {"step": 47, "t": 23.5, "odom": {"dx": 0.23322529259928318, "dy": -0.019104497838322406, "dtheta": -0.14192060040303678}, "gps": {"x": 10.088183336975103, "y": 2.5424346270421734}, "landmarks": [{"id": "3", "range": 4.187847256258201, "bearing": -3.1227642251070384}, {"id": "4", "range": 3.21598864316744, "bearing": 2.7253990620083997}, {"id": "5", "range": 3.141050525894102, "bearing": 2.174955950808889}]}, {"step": 48, "t": 24.0, "odom": {"dx": 0.198285138908359, "dy": -0.00814462662348614, "dtheta": -0.03856611876346921}, "gps": {"x": 10.199344965869184, "y": 2.143824982868909}, "landmarks": [{"id": "3", "range": 4.5293732888562355, "bearing": -3.0782604473726187}, {"id": "4", "range": 3.587135000773415, "bearing": 2.793853772184848}, {"id": "5", "range": 3.303975274882377, "bearing": 2.2044557551845845}]}, {"step": 49, "t": 24.5, "odom": {"dx": 0.23539935278350455, "dy": 0.005998029325142217, "dtheta": -0.09901552875976398}, "gps": {"x": 10.065970954624072, "y": 2.1289603073509795}, "landmarks": [{"id": "3", "range": 4.80930668376744, "bearing": -2.972110815448997}, {"id": "4", "range": 3.672778704132146, "bearing": 2.9250186093264308}, {"id": "5", "range": 3.081025638609965, "bearing": 2.4046275857140067}]}, {"step": 50, "t": 25.0, "odom": {"dx": 0.2473853307638995, "dy": -0.0247725358097061, "dtheta": -0.03167631399555215}, "gps": {"x": 10.608647836787453, "y": 2.00923358243315}, "landmarks": [{"id": "3", "range": 5.02905614458022, "bearing": -3.0027453605595045}, {"id": "4", "range": 3.8604552452666763, "bearing": 2.94684710191839}, {"id": "5", "range": 3.6398637698526346, "bearing": 2.4287399299614805}]}, {"step": 51, "t": 25.5, "odom": {"dx": 0.27174137030147144, "dy": -0.006705963368906786, "dtheta": 0.02969234568109566}, "gps": {"x": 10.750740791169342, "y": 1.62484307371623}, "landmarks": [{"id": "4", "range": 4.12659131809949, "bearing": 2.927168917571594}, {"id": "5", "range": 3.8120023027133962, "bearing": 2.4504653128619402}]}, {"step": 52, "t": 26.0, "odom": {"dx": 0.2742701733122944, "dy": -0.009686723088547783, "dtheta": 0.010222520897707588}, "gps": {"x": 10.639474033123218, "y": 1.5070431717656736}, "landmarks": [{"id": "4", "range": 4.298540038871713, "bearing": 2.9946854131691323}, {"id": "5", "range": 3.84028775292027, "bearing": 2.458942227278869}]}, {"step": 53, "t": 26.5, "odom": {"dx": 0.21357666326929434, "dy": 0.004328428729443719, "dtheta": -0.007667709554529526}, "gps": {"x": 11.0694419470034, "y": 1.4073385604751454}, "landmarks": [{"id": "4", "range": 4.480094787030317, "bearing": 2.9061812654786507}, {"id": "5", "range": 4.135559882832695, "bearing": 2.583669982963224}]}, {"step": 54, "t": 27.0, "odom": {"dx": 0.2380446583839066, "dy": 0.00996770271238575, "dtheta": 0.0577432926926794}, "gps": {"x": 11.247930704268386, "y": 1.1517699494615885}, "landmarks": [{"id": "4", "range": 4.897415200692686, "bearing": 2.9692933443242255}, {"id": "5", "range": 4.504524782464131, "bearing": 2.511629024826199}]}, {"step": 55, "t": 27.5, "odom": {"dx": 0.25542738054739955, "dy": 0.014411319757697924, "dtheta": 0.02736217285598344}, "gps": {"x": 11.013036076897068, "y": 0.7645526994532038}, "landmarks": [{"id": "5", "range": 4.656415969233047, "bearing": 2.5484657564513613}]}, {"step": 56, "t": 28.0, "odom": {"dx": 0.259688000200284, "dy": 0.011065322615745277, "dtheta": -0.05566159289406148}, "gps": {"x": 11.353583395999435, "y": 0.9573460635464699}, "landmarks": [{"id": "5", "range": 4.741677816340486, "bearing": 2.57671518082344}]}, {"step": 57, "t": 28.5, "odom": {"dx": 0.2523372377610979, "dy": -0.008412894164046992, "dtheta": -0.03485039952310765}, "gps": {"x": 11.528637900949828, "y": 0.520098272391447}, "landmarks": []}, {"step": 58, "t": 29.0, "odom": {"dx": 0.23722555751629684, "dy": -0.013867995908914469, "dtheta": 0.0226704961661397}, "gps": {"x": 11.714736026513094, "y": 0.26850000431198306}, "landmarks": []}, {"step": 59, "t": 29.5, "odom": {"dx": 0.2291360659757865, "dy": 0.0030407391222778586, "dtheta": 0.02540345055937542}, "gps": {"x": 11.806025655092684, "y": 0.16464216161815443}, "landmarks": []}]
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

## Revision 1 of 2: your previous solution.py
```python
import math
import numpy as np
import gtsam


def _angle_wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _make_gps_factor(pose_key: int, gx: float, gy: float, noise_model):
    def error_func(this: gtsam.CustomFactor, values: gtsam.Values, jacobians):
        pose: gtsam.Pose2 = values.atPose2(this.keys()[0])
        err = np.array([pose.x() - gx, pose.y() - gy], dtype=np.float64)
        if jacobians is not None:
            c = math.cos(pose.theta())
            s = math.sin(pose.theta())
            jacobians[0] = np.array([
                [c, -s, 0.0],
                [s,  c, 0.0],
            ], dtype=np.float64)
        return err

    return gtsam.CustomFactor(noise_model, [pose_key], error_func)


def _make_landmark_factor(pose_key: int, lx: float, ly: float, meas_range: float, meas_bearing: float, noise_model):
    def error_func(this: gtsam.CustomFactor, values: gtsam.Values, jacobians):
        pose: gtsam.Pose2 = values.atPose2(this.keys()[0])
        dx_w = lx - pose.x()
        dy_w = ly - pose.y()
        c = math.cos(pose.theta())
        s = math.sin(pose.theta())
        xb = c * dx_w + s * dy_w
        yb = -s * dx_w + c * dy_w
        r2 = max(xb * xb + yb * yb, 1e-12)
        r = math.sqrt(r2)
        b = math.atan2(yb, xb)
        err = np.array([r - meas_range, _angle_wrap(b - meas_bearing)], dtype=np.float64)
        if jacobians is not None:
            jacobians[0] = np.array([
                [-xb / r, -yb / r, 0.0],
                [yb / r2, -xb / r2, -1.0],
            ], dtype=np.float64)
        return err

    return gtsam.CustomFactor(noise_model, [pose_key], error_func)


def build_graph(measurements, public_spec, odom_scale: float = 1.0, prev_values: gtsam.Values = None):
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    init_spec = public_spec["initial_pose"]
    init_mean = init_spec["mean"]
    init_vars = init_spec["cov_diag_variances"]
    init_sigmas = np.sqrt(np.array(init_vars, dtype=np.float64))
    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(init_sigmas)

    x0_pose = gtsam.Pose2(float(init_mean[0]), float(init_mean[1]), float(init_mean[2]))
    graph.add(gtsam.PriorFactorPose2(0, x0_pose, prior_noise))

    noise_spec = public_spec["sensor_noise_std"]
    nom_odom_sigmas = np.array(noise_spec["odom"], dtype=np.float64)
    odom_noise = gtsam.noiseModel.Diagonal.Sigmas(nom_odom_sigmas * float(odom_scale))

    gps_sigma = float(noise_spec["gps"])
    gps_noise = gtsam.noiseModel.Isotropic.Sigma(2, gps_sigma)

    range_sigma = float(noise_spec["range"])
    bearing_sigma = float(noise_spec["bearing"])
    lm_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([range_sigma, bearing_sigma], dtype=np.float64))

    landmarks_spec = public_spec.get("landmarks", {})
    input_units = public_spec.get("input_units", {})
    pos_scale = 0.001 if input_units.get("odom", "m") == "mm" else 1.0
    ang_scale = math.pi / 180.0 if input_units.get("bearing", "rad") in ("deg", "degrees") else 1.0

    curr_pose = x0_pose
    for m in measurements:
        k = int(m["step"])
        if k == 0:
            if prev_values is not None and prev_values.exists(0):
                curr_pose = prev_values.atPose2(0)
            else:
                curr_pose = x0_pose
            initial.insert(0, curr_pose)
        else:
            odom = m["odom"]
            dx = float(odom["dx"]) * pos_scale
            dy = float(odom["dy"]) * pos_scale
            dtheta = float(odom["dtheta"]) * ang_scale
            rel_pose = gtsam.Pose2(dx, dy, dtheta)
            graph.add(gtsam.BetweenFactorPose2(k - 1, k, rel_pose, odom_noise))
            if prev_values is not None and prev_values.exists(k):
                curr_pose = prev_values.atPose2(k)
            else:
                prev_pose = initial.atPose2(k - 1)
                curr_pose = prev_pose.compose(rel_pose)
            initial.insert(k, curr_pose)

        gps = m.get("gps")
        if gps is not None:
            gx = float(gps["x"]) * pos_scale
            gy = float(gps["y"]) * pos_scale
            graph.add(_make_gps_factor(k, gx, gy, gps_noise))

        for lm in m.get("landmarks", []):
            lid = str(lm["id"])
            if lid in landmarks_spec:
                lx, ly = landmarks_spec[lid]
                r_meas = float(lm["range"]) * pos_scale
                b_meas = float(lm["bearing"]) * ang_scale
                graph.add(_make_landmark_factor(k, float(lx), float(ly), r_meas, b_meas, lm_noise))

    return graph, initial


class Estimator:
    def __init__(self, public_spec: dict):
        self.spec = public_spec
        self.measurements = []
        self.last_values = None
        scale_range = public_spec.get("odom_noise_scale_range", [1.0, 1.0])
        self.min_scale = float(scale_range[0])
        self.max_scale = float(scale_range[1])
        self.prior_scale = math.sqrt(self.min_scale * self.max_scale) if self.max_scale > self.min_scale else 1.0
        self.odom_scale = self.prior_scale

        input_units = public_spec.get("input_units", {})
        self.pos_scale = 0.001 if input_units.get("odom", "m") == "mm" else 1.0
        self.ang_scale = math.pi / 180.0 if input_units.get("bearing", "rad") in ("deg", "degrees") else 1.0
        self.nom_odom_sigmas = np.array(public_spec["sensor_noise_std"]["odom"], dtype=np.float64)
        self.nom_odom_inv_var = 1.0 / (self.nom_odom_sigmas ** 2)

    def _estimate_odom_scale(self, values: gtsam.Values, marginals: gtsam.Marginals) -> float:
        if self.max_scale <= self.min_scale or len(self.measurements) <= 1:
            return self.odom_scale

        k_steps = len(self.measurements) - 1
        # Use a mild Inverse-Gamma prior (equivalent to 3 steps = 9 degrees of freedom)
        dof_prior = 9.0
        sum_sq = dof_prior * (self.prior_scale ** 2)
        total_dof = dof_prior

        H1 = np.zeros((3, 3), dtype=np.float64, order="F")
        H2 = np.zeros((3, 3), dtype=np.float64, order="F")

        for idx in range(1, len(self.measurements)):
            m = self.measurements[idx]
            k = int(m["step"])
            odom = m["odom"]
            dx = float(odom["dx"]) * self.pos_scale
            dy = float(odom["dy"]) * self.pos_scale
            dtheta = float(odom["dtheta"]) * self.ang_scale
            meas_pose = gtsam.Pose2(dx, dy, dtheta)

            p_prev = values.atPose2(k - 1)
            p_curr = values.atPose2(k)
            pred_pose = p_prev.between(p_curr, H1, H2)
            err = meas_pose.localCoordinates(pred_pose)

            joint_cov = marginals.jointMarginalCovariance([k - 1, k]).fullMatrix()
            J = np.hstack([H1, H2])
            res_cov = J @ joint_cov @ J.T
            expected_sq = float(np.sum((err ** 2 + np.diag(res_cov)) * self.nom_odom_inv_var))
            sum_sq += expected_sq
            total_dof += 3.0

        est_scale = math.sqrt(max(sum_sq / total_dof, 1e-6))
        return float(np.clip(est_scale, self.min_scale, self.max_scale))

    def step(self, new_measurements: dict):
        self.measurements.append(new_measurements)
        k = int(new_measurements["step"])

        graph, initial = build_graph(
            self.measurements,
            self.spec,
            odom_scale=self.odom_scale,
            prev_values=self.last_values,
        )
        params = gtsam.LevenbergMarquardtParams()
        params.setRelativeErrorTol(1e-7)
        params.setAbsoluteErrorTol(1e-7)
        params.setMaxIterations(50)
        result = gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()
        marginals = gtsam.Marginals(graph, result)

        if self.max_scale > self.min_scale and k >= 2:
            new_scale = self._estimate_odom_scale(result, marginals)
            if abs(new_scale - self.odom_scale) / max(self.odom_scale, 1e-6) > 0.05:
                self.odom_scale = new_scale
                graph, initial = build_graph(
                    self.measurements,
                    self.spec,
                    odom_scale=self.odom_scale,
                    prev_values=result,
                )
                result = gtsam.LevenbergMarquardtOptimizer(graph, initial, params).optimize()
                marginals = gtsam.Marginals(graph, result)
            else:
                self.odom_scale = new_scale

        self.last_values = result
        pose_k = result.atPose2(k)
        cov_k = marginals.marginalCovariance(k)

        pose_out = [float(pose_k.x()), float(pose_k.y()), float(pose_k.theta())]
        cov_out = np.asarray(cov_k, dtype=np.float64)
        return pose_out, cov_out


def make_estimator(public_spec: dict) -> Estimator:
    return Estimator(public_spec)


def _dist_point_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    c1 = wx * vx + wy * vy
    if c1 <= 0.0:
        return math.hypot(px - x1, py - y1)
    c2 = vx * vx + vy * vy
    if c2 <= c1:
        return math.hypot(px - x2, py - y2)
    b = c1 / c2
    bx = x1 + b * vx
    by = y1 + b * vy
    return math.hypot(px - bx, py - by)


def _pure_pursuit_control(pose, waypoints, seg_idx: int, lookahead: float, v_cmd: float, omega_max: float):
    px, py, theta = pose
    n_seg = len(waypoints) - 1

    while seg_idx < n_seg - 1:
        wx, wy = waypoints[seg_idx + 1]
        if math.hypot(wx - px, wy - py) < lookahead:
            seg_idx += 1
        else:
            break

    x1, y1 = waypoints[seg_idx]
    x2, y2 = waypoints[seg_idx + 1]
    dx = x2 - x1
    dy = y2 - y1
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-12:
        tx, ty = x2, y2
    else:
        t_proj = ((px - x1) * dx + (py - y1) * dy) / seg_len2
        t_proj = max(0.0, min(1.0, t_proj))
        seg_len = math.sqrt(seg_len2)
        t_look = min(1.0, t_proj + lookahead / seg_len)
        tx = x1 + t_look * dx
        ty = y1 + t_look * dy

    dx_b = math.cos(theta) * (tx - px) + math.sin(theta) * (ty - py)
    dy_b = -math.sin(theta) * (tx - px) + math.cos(theta) * (ty - py)
    ld2 = max(dx_b * dx_b + dy_b * dy_b, 1e-6)
    curvature = 2.0 * dy_b / ld2
    omega = float(np.clip(v_cmd * curvature, -omega_max, omega_max))
    return v_cmd, omega, seg_idx


def _simulate_route_trial(route_waypoints, public_spec: dict, rng: np.random.Generator) -> bool:
    dt = float(public_spec["dt"])
    max_steps = int(public_spec["max_steps"])
    arrival_tol = float(public_spec["arrival_tolerance"])
    robot_radius = float(public_spec["robot_radius"])
    walls = public_spec.get("walls", [])
    landmarks = public_spec.get("landmarks", {})
    max_range = float(public_spec["landmark_visibility"]["max_range"])

    init_mean = np.array(public_spec["initial_pose"]["mean"], dtype=np.float64)
    init_stds = np.sqrt(np.array(public_spec["initial_pose"]["cov_diag_variances"], dtype=np.float64))
    true_pose = gtsam.Pose2(*rng.normal(init_mean, init_stds))

    scale_range = public_spec.get("odom_noise_scale_range", [1.0, 1.0])
    odom_scale = float(rng.uniform(scale_range[0], scale_range[1]))
    odom_stds = np.array(public_spec["sensor_noise_std"]["odom"], dtype=np.float64) * odom_scale
    gps_std = float(public_spec["sensor_noise_std"]["gps"])
    range_std = float(public_spec["sensor_noise_std"]["range"])
    bearing_std = float(public_spec["sensor_noise_std"]["bearing"])

    act_v_std = float(public_spec["actuator_noise_std"]["v"])
    act_w_std = float(public_spec["actuator_noise_std"]["omega"])

    ctrl = public_spec["controller"]
    lookahead = float(ctrl["lookahead"])
    v_nom = float(ctrl["v"])
    omega_max = float(ctrl["omega_max"])

    gps_sched = public_spec.get("gps_schedule", {"rate_steps": 1, "dropout_steps": []})
    rate_steps = int(gps_sched.get("rate_steps", 1))
    dropout = gps_sched.get("dropout_steps", [])
    has_dropout = len(dropout) == 2

    est = make_estimator(public_spec)
    goal_x, goal_y = route_waypoints[-1]
    seg_idx = 0
    prev_true_pose = true_pose

    for step in range(max_steps):
        if step == 0:
            odom_meas = None
        else:
            rel_true = prev_true_pose.between(true_pose)
            noise_xi = rng.normal(0.0, odom_stds)
            rel_noisy = rel_true.compose(gtsam.Pose2.Expmap(noise_xi))
            odom_meas = {
                "dx": float(rel_noisy.x()),
                "dy": float(rel_noisy.y()),
                "dtheta": float(rel_noisy.theta()),
            }

        in_dropout = has_dropout and (dropout[0] <= step <= dropout[1])
        if (step % rate_steps == 0) and not in_dropout:
            gps_meas = {
                "x": float(true_pose.x() + rng.normal(0.0, gps_std)),
                "y": float(true_pose.y() + rng.normal(0.0, gps_std)),
            }
        else:
            gps_meas = None

        lm_meas = []
        for lid, (lx, ly) in landmarks.items():
            dx_w = lx - true_pose.x()
            dy_w = ly - true_pose.y()
            r_true = math.hypot(dx_w, dy_w)
            if r_true <= max_range:
                b_true = _angle_wrap(math.atan2(dy_w, dx_w) - true_pose.theta())
                lm_meas.append({
                    "id": str(lid),
                    "range": float(r_true + rng.normal(0.0, range_std)),
                    "bearing": float(_angle_wrap(b_true + rng.normal(0.0, bearing_std))),
                })

        meas = {
            "step": step,
            "t": step * dt,
            "odom": odom_meas,
            "gps": gps_meas,
            "landmarks": lm_meas,
        }
        est_pose, _ = est.step(meas)

        for w in walls:
            if _dist_point_to_segment(true_pose.x(), true_pose.y(), w[0], w[1], w[2], w[3]) <= robot_radius:
                return False

        if math.hypot(true_pose.x() - goal_x, true_pose.y() - goal_y) <= arrival_tol:
            return True

        v_cmd, w_cmd, seg_idx = _pure_pursuit_control(
            est_pose, route_waypoints, seg_idx, lookahead, v_nom, omega_max
        )
        v_actual = v_cmd + rng.normal(0.0, act_v_std)
        w_actual = w_cmd + rng.normal(0.0, act_w_std)

        prev_true_pose = true_pose
        dtheta = w_actual * dt
        if abs(w_actual) < 1e-6:
            dx_b = v_actual * dt
            dy_b = 0.0
        else:
            dx_b = (v_actual / w_actual) * math.sin(dtheta)
            dy_b = (v_actual / w_actual) * (1.0 - math.cos(dtheta))
        true_pose = true_pose.compose(gtsam.Pose2(dx_b, dy_b, dtheta))

    return False


def choose_route(public_spec: dict) -> dict:
    routes = public_spec["routes"]
    rng = np.random.default_rng(42)
    n_trials = 25

    results = {}
    for route_name, waypoints in routes.items():
        successes = sum(_simulate_route_trial(waypoints, public_spec, rng) for _ in range(n_trials))
        results[route_name] = successes / float(n_trials)

    best_route = max(results.keys(), key=lambda r: (results[r], -len(routes[r])))
    best_prob = float(results[best_route])

    reasoning = (
        f"Route B avoids the narrow wall corridor at y=+-0.45 (x in [5.4, 9.0]) and stays within range "
        f"of the landmarks at y=5.5 during the GPS dropout (steps 20-35), whereas Route A passes directly "
        f"through the narrow corridor with neither GPS nor landmark measurements. "
        f"Closed-loop Monte Carlo success probabilities: A={results.get('A', 0.0):.2f}, B={results.get('B', 0.0):.2f}."
    )
    return {
        "route": best_route,
        "success_prob": best_prob,
        "reasoning": reasoning,
    }

```

## Verifier feedback on that solution
- Calibration at end of trajectory: overconfident. Only 24/30 runs had NEES <= 7.81 (binomial p = 0.0033 < 0.01). Covariance too small (mean NEES 3.92, expected about 3).

Fix the failed checks. Return the complete revised solution.py.