
--library IEEE;
--use IEEE.STD_LOGIC_1164.ALL;
--use IEEE.NUMERIC_STD.ALL;

--entity hazard_detection is

--    generic (
--        WIDTH  : integer := 640;
--        HEIGHT : integer := 480;

--        THRESHOLD : integer := 80;
--        MIN_HAZARD_PIXELS : integer := 10;

--        RED_THICKNESS : integer := 3;

--        GREEN_X0 : integer := 220;
--        GREEN_X1 : integer := 420;
--        GREEN_Y0 : integer := 150;
--        GREEN_Y1 : integer := 350;

--        GREEN_THICKNESS : integer := 3
--    );

--    port (
--        clk : in std_logic;
--        reset : in std_logic;

--        pixel_in : in std_logic_vector(23 downto 0);

--        pixel_x : in unsigned(10 downto 0);
--        pixel_y : in unsigned(10 downto 0);

--        de          : in std_logic;
--        hsync_in    : in std_logic;
--        vsync_in    : in std_logic;
--        frame_start : in std_logic;

--        pixel_out : out std_logic_vector(23 downto 0);

--        de_out    : out std_logic;
--        hsync_out : out std_logic;
--        vsync_out : out std_logic
--    );

--end hazard_detection;


--architecture Behavioral of hazard_detection is

--    -- Current frame boundaries

--    signal min_x : integer range 0 to WIDTH-1 := WIDTH-1;
--    signal max_x : integer range 0 to WIDTH-1 := 0;

--    signal min_y : integer range 0 to HEIGHT-1 := HEIGHT-1;
--    signal max_y : integer range 0 to HEIGHT-1 := 0;

--    signal hazard_count : integer range 0 to WIDTH*HEIGHT := 0;

--    -- Previous completed frame boundaries

--    signal active_min_x : integer range 0 to WIDTH-1 := 0;
--    signal active_max_x : integer range 0 to WIDTH-1 := 0;

--    signal active_min_y : integer range 0 to HEIGHT-1 := 0;
--    signal active_max_y : integer range 0 to HEIGHT-1 := 0;

--    signal hazard_valid : std_logic := '0';

--begin

--    ------------------------------------------------
--    -- VIDEO TIMING OUTPUTS
--    ------------------------------------------------

--    de_out    <= de;
--    hsync_out <= hsync_in;
--    vsync_out <= vsync_in;


--    ------------------------------------------------
--    -- HAZARD DETECTION PROCESS
--    ------------------------------------------------

--    process(clk)

--        variable r : integer;
--        variable g : integer;
--        variable b : integer;

--        variable gray : integer;

--        variable x : integer;
--        variable y : integer;

--    begin

--        if rising_edge(clk) then

--            if reset = '1' then

--                min_x <= WIDTH-1;
--                max_x <= 0;

--                min_y <= HEIGHT-1;
--                max_y <= 0;

--                hazard_count <= 0;

--                active_min_x <= 0;
--                active_max_x <= 0;

--                active_min_y <= 0;
--                active_max_y <= 0;

--                hazard_valid <= '0';

--            elsif frame_start = '1' then

--                -- Save completed frame boundaries

--                if hazard_count >= MIN_HAZARD_PIXELS then

--                    active_min_x <= min_x;
--                    active_max_x <= max_x;

--                    active_min_y <= min_y;
--                    active_max_y <= max_y;

--                    hazard_valid <= '1';

--                else

--                    hazard_valid <= '0';

--                end if;

--                -- Reset statistics for next frame

--                min_x <= WIDTH-1;
--                max_x <= 0;

--                min_y <= HEIGHT-1;
--                max_y <= 0;

--                hazard_count <= 0;

--            elsif de = '1' then

--                -- Extract RGB

--                r := to_integer(unsigned(pixel_in(23 downto 16)));
--                g := to_integer(unsigned(pixel_in(15 downto 8)));
--                b := to_integer(unsigned(pixel_in(7 downto 0)));

--                -- Grayscale conversion

--                gray := (77*r + 150*g + 29*b) / 256;

--                x := to_integer(pixel_x);
--                y := to_integer(pixel_y);

--                -- Threshold detection

--                if gray < THRESHOLD then

--                    if x < WIDTH and y < HEIGHT then

--                        if x < min_x then
--                            min_x <= x;
--                        end if;

--                        if x > max_x then
--                            max_x <= x;
--                        end if;

--                        if y < min_y then
--                            min_y <= y;
--                        end if;

--                        if y > max_y then
--                            max_y <= y;
--                        end if;

--                        if hazard_count < WIDTH*HEIGHT then
--                            hazard_count <= hazard_count + 1;
--                        end if;

--                    end if;

--                end if;

--            end if;

--        end if;

--    end process;


--    ------------------------------------------------
--    -- VIDEO OVERLAY PROCESS
--    ------------------------------------------------
--process(
--    pixel_x,
--    pixel_y,
--    pixel_in,
--    de
--)
--begin

--    pixel_out <= pixel_in;

--    if de = '1' then
--        pixel_out <= x"00FF00";
--    end if;

--end process;
----    process(
----        pixel_x,
----        pixel_y,
----        pixel_in,
----        de,
----        hazard_valid,
----        active_min_x,
----        active_max_x,
----        active_min_y,
----        active_max_y
----    )

----        variable x : integer;
----        variable y : integer;

----        variable red_border : boolean;
----        variable green_border : boolean;

----        variable output_pixel : std_logic_vector(23 downto 0);

----        variable red_left : integer;
----        variable red_right : integer;
----        variable red_top : integer;
----        variable red_bottom : integer;

----    begin

----        x := to_integer(pixel_x);
----        y := to_integer(pixel_y);

----        output_pixel := pixel_in;

----        red_border := false;
----        green_border := false;

----        -- Calculate protected red rectangle boundaries

----        red_left := active_min_x - RED_THICKNESS;
----        red_right := active_max_x + RED_THICKNESS;

----        red_top := active_min_y - RED_THICKNESS;
----        red_bottom := active_max_y + RED_THICKNESS;

----        if red_left < 0 then
----            red_left := 0;
----        end if;

----        if red_right >= WIDTH then
----            red_right := WIDTH-1;
----        end if;

----        if red_top < 0 then
----            red_top := 0;
----        end if;

----        if red_bottom >= HEIGHT then
----            red_bottom := HEIGHT-1;
----        end if;


--        ------------------------------------------------
----        -- RED HAZARD OUTLINE
----        ------------------------------------------------

----        if hazard_valid = '1' then

----            if x >= red_left and x <= red_right and
----               y >= red_top and y <= red_bottom then

----                if x <= red_left + RED_THICKNESS or
----                   x >= red_right - RED_THICKNESS or
----                   y <= red_top + RED_THICKNESS or
----                   y >= red_bottom - RED_THICKNESS then

----                    red_border := true;

----                end if;

----            end if;

----        end if;


--        ------------------------------------------------
--        -- GREEN LANDING ZONE OUTLINE
--        ------------------------------------------------

----        if x >= GREEN_X0 and x <= GREEN_X1 and
----           y >= GREEN_Y0 and y <= GREEN_Y1 then

----            if x <= GREEN_X0 + GREEN_THICKNESS or
----               x >= GREEN_X1 - GREEN_THICKNESS or
----               y <= GREEN_Y0 + GREEN_THICKNESS or
----               y >= GREEN_Y1 - GREEN_THICKNESS then

----                green_border := true;

----            end if;

----        end if;


--        ------------------------------------------------
--        -- SELECT OUTPUT COLOR
--        ------------------------------------------------

----        if de = '1' then

----            if red_border then

----                output_pixel := x"FF0000";

----            elsif green_border then

----                output_pixel := x"00FF00";

----            end if;

----        end if;

----        pixel_out <= output_pixel;

----    end process;
----if de = '1' then

----    if x >= 100 and x <= 300 and
----       y >= 100 and y <= 300 then

----        output_pixel := x"00FF00";

----    else

----        output_pixel := pixel_in;

----    end if;

----end if;

----pixel_out <= output_pixel;

--end Behavioral;
