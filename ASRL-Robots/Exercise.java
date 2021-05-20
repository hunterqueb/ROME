public class Exercise
{
   public static void main(String [] args)
   {
      Monster m = new Worgen("Tess Greymane", PronounSet.FEMALE());

      if (m.instanceOf(Worgen))
         (Worgen)m.howlAtTheMoon();
   }
}
