with Gcode_Parser;   use Gcode_Parser;
with Motion_Planner; use Motion_Planner;
with Motion_Planner.Planner;
with Physical_Types; use Physical_Types;
with Ada.Text_IO;    use Ada.Text_IO;
with Ada.Command_Line;
with GNAT.OS_Lib;
with Ada.Exceptions;
with System.Dim.Float_IO;

package body Main is

   package Dimensioed_Float_IO is new System.Dim.Float_IO (Dimensioned_Float);
   use Dimensioed_Float_IO;

   package Planner is new Motion_Planner.Planner (Boolean, False);
   use Planner;

   Limits : constant Motion_Planner.Kinematic_Limits :=
     (Velocity_Max     => 100.0 * mm / s,
      Acceleration_Max => 1_500.0 * mm / s**2,
      Jerk_Max         => 2_500_000.0 * mm / s**3,
      Snap_Max         => 0.8 * 500_000_000.0 * mm / s**4, --  Jm / Ts
      Crackle_Max      => 0.8 * 500_000_000_000.0 * mm / s**5, --  Jm / Ts**2
      Chord_Error_Max  => 0.1 * mm);

   task Reader is
      entry Start (Filename : String);
   end Reader;

   task body Reader is
      F              : File_Type;
      C              : Gcode_Parser.Command := (others => <>);
      Parser_Context : Gcode_Parser.Context := Make_Context ([others => 0.0 * mm], 100.0 * mm / s);
   begin
      accept Start (Filename : String) do
         Open (F, In_File, Filename);
      end Start;

      while not End_Of_File (F) loop
         begin
            Parse_Line (Parser_Context, Get_Line (F), C);
            if C.Kind = Move_Kind then
               C.Pos (E_Axis) := 0.0 * mm;
               Planner.Enqueue ((Kind => Planner.Move_Kind, Pos => C.Pos * [others => 1.0], Limits => Limits));
            end if;
         exception
            when Bad_Line =>
               null;
         end;
      end loop;

      Planner.Enqueue ((Kind => Planner.Flush_Kind, Flush_Extra_Data => True));
   exception
      when E : others =>
         Put_Line ("Error in file reader: ");
         Put_Line (Ada.Exceptions.Exception_Information (E));
   end Reader;

   Block : Planner.Execution_Block;

   procedure Main is
      Total_Time            : Time   := 0.0 * s;
      Total_Corner_Distance : Length := 0.0 * mm;
   begin
      if Ada.Command_Line.Argument_Count /= 1 then
         Put_Line ("Provide exactly 1 command line argument.");
         GNAT.OS_Lib.OS_Exit (1);
      end if;

      Reader.Start (Ada.Command_Line.Argument (1));

      loop
         Planner.Dequeue (Block);

         for I in Block.Feedrate_Profiles'Range loop
            Total_Time            := Total_Time + Motion_Planner.Total_Time (Block.Feedrate_Profiles (I));
            Total_Corner_Distance := Total_Corner_Distance + abs (Block.Corners (I) - Block.Corners (I - 1));
         end loop;

         exit when Block.Flush_Extra_Data;
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
