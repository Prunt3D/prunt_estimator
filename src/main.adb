with Prunt;                use Prunt;
with Prunt.Motion_Planner; use Prunt.Motion_Planner;
with Prunt.Motion_Planner.Planner;
with Prunt.Gcode_Parser;   use Prunt.Gcode_Parser;
with Ada.Text_IO;          use Ada.Text_IO;
with Ada.Command_Line;
with GNAT.OS_Lib;
with Ada.Exceptions;
with System.Dim.Float_IO;

package body Main is

   package Dimensioed_Float_IO is new System.Dim.Float_IO (Dimensioned_Float);
   use Dimensioed_Float_IO;

   type Null_Record is null record;

   function Is_Homing_Move (Data : Boolean) return Boolean is
   begin
      return False;
   end Is_Homing_Move;

   package My_Planner is new Motion_Planner.Planner
     (Flush_Extra_Data_Type        => Boolean,
      Flush_Extra_Data_Default     => False,
      Corner_Extra_Data_Type       => Null_Record,
      Initial_Position             => [others => 0.0 * mm],
      Max_Corners                  => 3_000,
      Is_Homing_Move               => Is_Homing_Move,
      Home_Move_Minimum_Coast_Time => 0.0 * s);

   Limits : constant Motion_Planner.Kinematic_Parameters :=
     (Lower_Pos_Limit         => (others => Length'First),
      Upper_Pos_Limit         => (others => Length'Last),
      Ignore_E_In_XYZE        => True,
      Shift_Blended_Corners   => True,
      Tangential_Velocity_Max => 10_000_000_000.0 * mm / s,
      Axial_Velocity_Maxes    =>
        (X_Axis => 100.0 * mm / s, Y_Axis => 100.0 * mm / s, Z_Axis => 50.0 * mm / s, E_Axis => 15.0 * mm / s),
      Pressure_Advance_Time   => 0.0 * s,
      Acceleration_Max        => 50_000.0 * mm / s**2,
      Jerk_Max                => 226_000.0 * mm / s**3,
      --  Snap_Max                => 2_000_000.0 * mm / s**4,
      --  Acceleration_Max        => 2_000.0 * mm / s**2,
      --  Jerk_Max                => 1_000_000_000_000.0 * mm / s**3,
      Snap_Max                => 1_000_000_000_000.0 * mm / s**4,
      Crackle_Max             => 1_000_000_000_000.0 * mm / s**5,
      Chord_Error_Max         => 0.1 * mm,
      Axial_Scaler            => (X_Axis => 1.0, Y_Axis => 1.0, Z_Axis => 0.2, E_Axis => 0.1));

   task Reader is
      entry Start (Filename : String);
   end Reader;

   task body Reader is
      F              : File_Type;
      C              : Gcode_Parser.Command;
      Parser_Context : Gcode_Parser.Context := Make_Context ((others => 0.0 * mm), 100.0 * mm / s);
   begin
      accept Start (Filename : String) do
         Open (F, In_File, Filename);
      end Start;

      while not End_Of_File (F) loop
         declare
            Line : String := Get_Line (F);
         begin
            Parse_Line (Parser_Context, Line, C);
            if C.Kind = Move_Kind then
               My_Planner.Enqueue
                 ((Kind             => My_Planner.Move_Kind,
                   Pos              => C.Pos,
                   Feedrate         => C.Feedrate,
                  Corner_Extra_Data => (null record)));
            end if;
         exception
            when E : Bad_Line =>
               Put_Line ("Error on line:");
               Put_Line (Line);
               Put_Line (Ada.Exceptions.Exception_Information (E));
               raise Bad_Line;
         end;
      end loop;

      My_Planner.Enqueue ((Kind => My_Planner.Flush_Kind, Flush_Extra_Data => True));
   exception
      when E : others =>
         Put_Line ("Error in file reader: ");
         Put_Line (Ada.Exceptions.Exception_Information (E));
   end Reader;

   Block : My_Planner.Execution_Block;

   procedure Main is
      Total_Time            : Time   := 0.0 * s;
      Total_Corner_Distance : Length := 0.0 * mm;
   begin
      if Ada.Command_Line.Argument_Count /= 1 then
         Put_Line ("Provide exactly 1 command line argument.");
         GNAT.OS_Lib.OS_Exit (1);
      end if;

      My_Planner.Runner.Setup (Limits);

      Reader.Start (Ada.Command_Line.Argument (1));

      loop
         My_Planner.Dequeue (Block);

         for I in 2 .. Block.N_Corners loop
            Total_Time            := Total_Time + My_Planner.Segment_Time (Block, I);
            Total_Corner_Distance := Total_Corner_Distance + My_Planner.Segment_Corner_Distance (Block, I);
         end loop;

         exit when My_Planner.Flush_Extra_Data (Block);
      end loop;

      Put ("Total Time: ");
      Put (Total_Time, Aft => 3, Exp => 0);
      Put_Line ("");
      Put ("Total Corner Distance: ");
      Put (Total_Corner_Distance, Aft => 3, Exp => 0);
      Put_Line ("");

      GNAT.OS_Lib.OS_Exit (0);
   end Main;

end Main;
