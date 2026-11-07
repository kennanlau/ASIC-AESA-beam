`default_nettype none
// Beam steering engine for a 4x4 array at half-wavelength spacing.
// Element n = iy*4 + ix. Phase in 7-bit steps of 2.8125 deg (ADAR1000 resolution):
//   phase(n) = floor( (8 - (ix*Dx + iy*Dy)) / 16 ) mod 128
// Dx, Dy are signed 12-bit phase increments in 1/16 steps. In angle mode,
//   D = round(1024 * sin(theta)), i.e. 64*sin(theta) steps with 4 fractional bits.
module beam_steer (
    input  wire       clk,
    input  wire       rst,
    input  wire       wr,        // 1-cycle pulse
    input  wire [2:0] sel,
    input  wire [7:0] din,
    input  wire [3:0] n,         // element index to read
    output reg  [6:0] phase,     // registered phase of element n
    output reg        ack        // toggles on every accepted write
);
    reg  signed [11:0] dx, dy;

    // angle mode: din is a signed angle in degrees, clamped to +/-90
    wire               neg_a = din[7];
    wire [7:0]         mag8  = neg_a ? (~din + 8'd1) : din;
    wire [6:0]         deg   = (mag8 > 8'd90) ? 7'd90 : mag8[6:0];
    wire [10:0]        lut;
    sin_lut u_lut (.deg(deg), .lut(lut));
    wire signed [11:0] pos_a = $signed({1'b0, lut});
    wire signed [11:0] d_ang = neg_a ? -pos_a : pos_a;

    always @(posedge clk) begin
        if (rst) begin
            dx <= 12'sd0; dy <= 12'sd0; ack <= 1'b0;
        end else if (wr) begin
            ack <= ~ack;
            case (sel)
                3'd0: dx[7:0]  <= din;
                3'd1: dx[11:8] <= din[3:0];
                3'd2: dy[7:0]  <= din;
                3'd3: dy[11:8] <= din[3:0];
                3'd4: dx <= d_ang;
                3'd5: dy <= d_ang;
                3'd6: begin dx <= 12'sd0; dy <= 12'sd0; end
                default: ;
            endcase
        end
    end

    // i * D for i = 0..3 using shifts and one add
    function signed [14:0] times;
        input signed [11:0] d;
        input        [1:0]  i;
        begin
            case (i)
                2'd0:    times = 15'sd0;
                2'd1:    times = {{3{d[11]}}, d};
                2'd2:    times = {{2{d[11]}}, d, 1'b0};
                default: times = {{2{d[11]}}, d, 1'b0} + {{3{d[11]}}, d};
            endcase
        end
    endfunction

    wire signed [14:0] acc = times(dx, n[1:0]) + times(dy, n[3:2]);
    wire signed [14:0] r   = 15'sd8 - acc;             // round half up, then floor by >>4
    wire               _unused_r = &{r[14:11], r[3:0], 1'b0};   // only r[10:4] is the phase

    always @(posedge clk) begin
        if (rst) phase <= 7'd0;
        else     phase <= r[10:4];                       // (r >>> 4) mod 128
    end
endmodule
