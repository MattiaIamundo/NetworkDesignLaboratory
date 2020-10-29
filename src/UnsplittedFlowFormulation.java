import java.util.*;

import com.jom.OptimizationProblem;
import com.net2plan.interfaces.networkDesign.Demand;
import com.net2plan.interfaces.networkDesign.IAlgorithm;
import com.net2plan.interfaces.networkDesign.Link;
import com.net2plan.interfaces.networkDesign.Net2PlanException;
import com.net2plan.interfaces.networkDesign.NetPlan;
import com.net2plan.interfaces.networkDesign.Node;
import com.net2plan.utils.Constants.RoutingType;
import com.net2plan.utils.Triple;

import cern.colt.matrix.tdouble.DoubleMatrix2D;

public class UnsplittedFlowFormulation implements IAlgorithm{
    public String executeAlgorithm(NetPlan netPlan, Map<String, String> algorithmParameters, Map<String, String> net2planParameters)
    {
        /* Remove all the carried traffic in the network*/
        netPlan.removeAllUnicastRoutingInformation();
        OptimizationProblem op = new OptimizationProblem ();
        /* add the vector of decision variables */
        final int D = netPlan.getNumberOfDemands();
        final int E = netPlan.getNumberOfLinks ();
        op.addDecisionVariable("x_lkc" , true , new int [] {D , E } , 0 , 1);
        op.addDecisionVariable("V_lkc", false, new int[] {D, E}, 0, Double.MAX_VALUE);

        for (Link lk : netPlan.getLinks()) {
            op.setInputParameter("lk" , lk.getIndex ());
            for (Demand d : netPlan.getDemands()){
                op.setInputParameter ("c" , d.getIndex ());
                op.setInputParameter("v_c", d.getOfferedTraffic());
                op.addConstraint("V_lkc(c, lk) == x_lkc(c, lk) * v_c");
            }
        }

        double [] connections = new double[D];
        double [] conIndex = new double[D];
        for (Demand d : netPlan.getDemands()){
            connections[d.getIndex()] = d.getOfferedTraffic();
            conIndex[d.getIndex()] = d.getIndex();
        }

        op.setInputParameter("v_c", connections, "row");
        op.setInputParameter("con", conIndex, "row");


        op.setObjectiveFunction("minimize" , "sum(V_lkc)");

        // Add the solenoidality constraints
        for (Node n : netPlan.getNodes ())
        {
            op.setInputParameter ("IncomingLinks" , NetPlan.getIndexes (n.getIncomingLinks()) , "row");
            op.setInputParameter ("OutgoingLinks" , NetPlan.getIndexes (n.getOutgoingLinks()) , "row");
            for (Demand d : netPlan.getDemands ())
            {
                op.setInputParameter ("c" , d.getIndex ());
                if (n == d.getIngressNode())
                    op.addConstraint ("sum(x_lkc(c,OutgoingLinks)) - sum(x_lkc(c,IncomingLinks)) == 1");
                else if (n == d.getEgressNode())
                    op.addConstraint ("sum(x_lkc(c,OutgoingLinks)) - sum(x_lkc(c,IncomingLinks)) == -1");
                else
                    op.addConstraint ("sum(x_lkc(c,OutgoingLinks)) - sum(x_lkc(c,IncomingLinks)) == 0");
            }
        }

        // Add the link capacity constraints
        for (Link lk : netPlan.getLinks ()) {
            op.setInputParameter("lk" , lk.getIndex ());
            op.setInputParameter("linkCapacity" , lk.getCapacity ());
            op.addConstraint ("sum(V_lkc(all, lk)) <= linkCapacity");
        }

        /* call the solver to solve the problem */
        op.solve ("glpk");

        /* An optimal solution was not found */
        if (!op.solutionIsOptimal())
            throw new Net2PlanException ("An optimal solution was not found");


        final DoubleMatrix2D x_lkc = op.getPrimalSolution("x_lkc").view2D();
        Set<Demand> demands = new HashSet<Demand>(netPlan.getDemands());
        netPlan.setRoutingFromDemandLinkCarriedTraffic(x_lkc , true , false, demands, netPlan.getNetworkLayerDefault());
        return "Total carried traffic in the links: " + netPlan.getVectorLinkCarriedTraffic().zSum();
    }

    @Override
    public String getDescription()
    {
        return "Flow Formulation Constraints";
    }


    @Override
    public List<Triple<String, String, String>> getParameters()
    {
        final List<Triple<String, String, String>> param = new LinkedList<Triple<String, String, String>> ();
        return param;
    }
}
